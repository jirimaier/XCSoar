#include "Device/Error.hpp"
#include "Device/Port/Port.hpp"
#include "Device/RecordedFlight.hpp"
#include "LXEosDevice.hpp"
#include "Operation/Operation.hpp"
#include "io/BufferedOutputStream.hxx"
#include "io/FileOutputStream.hxx"
#include "system/Path.hpp"
#include "util/AllocatedArray.hxx"
#include "util/ByteOrder.hxx"
#include "util/ScopeExit.hxx"

bool
LXEosDevice::ReadFlightList(RecordedFlightList& flight_list,
                            OperationEnvironment& env)
{
  port.StopRxThread();

  uint8_t flight_count = 0;

  env.SetProgressRange(1);
  env.SetProgressPosition(0);

  bool success = GetNumberOfFlights(flight_count, env);

  env.SetProgressRange(flight_count + 1);
  env.SetProgressPosition(1);

  if (success) {
    for (uint8_t i = 0; i < flight_count; i++) {
      RecordedFlightInfo flight;

      // Try up to 5 times before giving up
      for (uint8_t attempt = 0; attempt < 5; attempt++) {
        success = GetFlightInfo(i + 1, flight, env);
        if (success)
          break;
      }

      if (!success)
        break;

      flight_list.append(flight);
      env.SetProgressPosition(i + 2);
    }
  }

  env.SetProgressPosition(flight_count + 1);

  port.StartRxThread();
  return success;
}

bool
LXEosDevice::DownloadFlight(const RecordedFlightInfo& flight,
                            Path path,
                            OperationEnvironment& env)
{
  FileOutputStream fos(path);
  BufferedOutputStream bos(fos);

  bool success = true;

  port.StopRxThread();

  uint16_t flight_id = flight.internal.lx_eos.flight_id;
  uint32_t bytes_remaining = flight.internal.lx_eos.file_size;

  env.SetProgressRange(100);
  env.SetProgressPosition(0);

  // Download blocks until remaining bytes reach zero
  for (uint16_t block_id = 0; bytes_remaining > 0; block_id++) {
    AllocatedArray<std::byte> block;

    // Try up to 5 times before giving up
    for (uint8_t attempt = 0; attempt < 5; attempt++) {
      success = GetFlightLogBlock(block, flight_id, block_id, env);
      if (success)
        break;
    }

    if (!success)
      break;

    bos.Write(block);

    // Make sure that bytes remaining cannot underflow
    if(block.size() > bytes_remaining)
      return false;
    
    bytes_remaining -= block.size();

    // Progress in percents downloaded
    float progress =
      100.0f * (1.0f - static_cast<float>(bytes_remaining) /
                         static_cast<float>(flight.internal.lx_eos.file_size));
    env.SetProgressPosition(static_cast<unsigned int>(floor(progress)));
  }

  port.Flush();

  if (success) {
    bos.Flush();
    fos.Commit();
  }

  env.SetProgressPosition(100);

  port.StartRxThread();
  return success;
}

bool
LXEosDevice::GetNumberOfFlights(uint8_t& flight_count,
                                OperationEnvironment& env)
{
  EosGetNumOfFlights data;
  std::byte response[3];
  response[0] = std::byte{ 0x06 };

  auto data_array = reinterpret_cast<std::byte*>(&data);
  std::span<std::byte> message(data_array, sizeof(data));

  if (!WriteAndWaitForACK(message, env))
    return false;

  try {
    port.FullRead({ response + 1, sizeof(response) - 1 },
                  env,
                  std::chrono::milliseconds(5000));
  } catch (...) {
    return false;
  }

  // Check CRC
  if (CalculateCRC(response, sizeof(response), 0xFF) != 0x00)
    return false;

  flight_count = std::to_integer<uint8_t>(response[1]);
  return true;
}

bool
LXEosDevice::GetFlightInfo(const uint8_t index,
                           RecordedFlightInfo& flight,
                           OperationEnvironment& env)
{
  if (index == 0) // Invalid
    return false;

  EosRequestFlightInfo data;
  data.flight_id = index;
  std::byte response[94];

  auto data_array = reinterpret_cast<std::byte*>(&data);
  data.crc = CalculateCRC(data_array, sizeof(data) - 1, 0xFF);
  std::span<std::byte> message(data_array, sizeof(data));

  if (!WriteAndWaitForACK(message, env))
    return false;
  response[0] = std::byte{ 0x06 };

  try {
    port.FullRead({ response + 1, sizeof(response) - 1 },
                  env,
                  std::chrono::milliseconds(5000));
  } catch (...) {
    return false;
  }

  if (CalculateCRC(response, sizeof(response), 0xFF) != 0x00) // Check CRC
    return false;

  /* Flight ID for downloading is the index (1 = newest),
  not the ID from this message */
  flight.internal.lx_eos.flight_id = index;

  uint32_t julianDate, time_takeoff, time_landing, file_size;
  std::memcpy(&julianDate, &response[13], 4);
  std::memcpy(&time_takeoff, &response[17], 4);
  std::memcpy(&time_landing, &response[21], 4);
  std::memcpy(&file_size, &response[89], 4);
  flight.date = julianToDate(julianDate);
  flight.start_time = BrokenTime::FromSecondOfDay(time_takeoff);
  flight.end_time = BrokenTime::FromSecondOfDay(time_landing);
  flight.internal.lx_eos.file_size = file_size;
  return true;
}

bool
LXEosDevice::GetFlightLogBlock(AllocatedArray<std::byte>& block,
                               uint16_t flight_id,
                               uint16_t block_id,
                               OperationEnvironment& env)
{
  EosRequestFlightBlock data;
  data.flight_id = flight_id;
  data.block_id = block_id;
  std::byte responseHeader[5];
  responseHeader[0] = std::byte{ 0x06 };

  auto data_array = reinterpret_cast<std::byte*>(&data);
  data.crc = CalculateCRC(data_array, sizeof(data) - 1, 0xFF);
  std::span<std::byte> message(data_array, sizeof(data));

  if (!WriteAndWaitForACK(message, env))
    return false;

  try {
    port.FullRead(std::span{ responseHeader + 1, sizeof(responseHeader) - 1 },
                  env,
                  std::chrono::milliseconds(5000));
  } catch (...) {
    return false;
  }

  // Read size and ID of received block
  uint16_t size, id;
  std::memcpy(&size, &responseHeader[1], 2);
  std::memcpy(&id, &responseHeader[3], 2);

  // Check that received ID matches the requested one
  if (id != block_id)
    return false;

  block.ResizeDiscard(size);
  try {
    port.FullRead(
      { block.data(), block.size() }, env, std::chrono::milliseconds(5000));
  } catch (...) {
    return false;
  }

  std::byte crc_byte[1];
  try {
    port.WaitAndRead(crc_byte, env, std::chrono::milliseconds(5000));
  } catch (...) {
    return false;
  }

  // Message has 3 parts: header, block, and CRC byte.
  // Calculate CRC from all 3 parts, result should be 0x00
  uint8_t crc = CalculateCRC(responseHeader, sizeof(responseHeader), 0xFF);
  crc = CalculateCRC(block.data(), block.size(), crc);
  if (CalculateCRC(crc_byte, 1, crc) != 0)
    return false;

  return true;
}

BrokenDate
LXEosDevice::julianToDate(uint32_t julian_date)
{
  // Convert Julian date BrokenDate
  BrokenDate bd;
  int a = julian_date + 32044;
  int b = (4 * a + 3) / 146097;
  int c = a - (146097 * b) / 4;
  int d = (4 * c + 3) / 1461;
  int e = c - (1461 * d) / 4;
  int m = (5 * e + 2) / 153;

  bd.day = e - (153 * m + 2) / 5 + 1;
  bd.month = m + 3 - 12 * (m / 10);
  bd.year = 100 * b + d - 4800 + (m / 10);
  bd.day_of_week = -1; // No need to determine it

  return bd;
}