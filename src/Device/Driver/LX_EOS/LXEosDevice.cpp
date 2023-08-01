#include "LXEosDevice.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/DeviceInfo.hpp"
#include "NMEA/Info.hpp"
#include "util/ByteOrder.hxx"

void
LXEosDevice::LinkTimeout()
{
  const std::lock_guard lock{ settings_mutex };
  vario_settings.uptodate = false;
}

bool
LXEosDevice::EnableNMEA(OperationEnvironment& env)
{
  /**
   * Set up the NMEA sentences sent by the vario:
   *
   * - LXWP0 every second (most important data)
   * - LXWP1 every 60 seconds (device info)
   * - LXWP2 every 57 seconds (MC, Bugs, Ballast settings)
   * - LXWP3 every 17 seconds (only used for QNH)
   *
   * It seems that LXWP3 does not get sent if it is to be sent at the same time
   * as LXWP2, so choosing odd numbers to reduce overlaps
   *
   * LXWP2 is also sent automatically whenever vario settings are changed by
   * pilot. Periodical sending is there only as a backup sync method.
   */

  PortWriteNMEA(port, "PFLX0,LXWP0,1,LXWP1,60,LXWP2,11,LXWP3,17", env);

  port.Flush();

  return true;
}

bool
LXEosDevice::PutMacCready(double mc, OperationEnvironment& env)
{
  const std::lock_guard lock{ settings_mutex };
  vario_settings.mc = mc;
  return SendNewSettings(env);
}

bool
LXEosDevice::PutBugs(double bugs, OperationEnvironment& env)
{
  const std::lock_guard lock{ settings_mutex };
  vario_settings.bugs = (1.0 - bugs) * 100.0;
  return SendNewSettings(env);
}

bool
LXEosDevice::ParseNMEA(const char* String, NMEAInfo& info)
{
  if (!VerifyNMEAChecksum(String))
    return false;

  NMEAInputLine line(String);

  const auto type = line.ReadView();
  if (type == "$LXWP0"sv)
    return LXWP0(line, info);
  else if (type == "$LXWP1"sv)
    return LXWP1(line, info.device);
  else if (type == "$LXWP2"sv)
    return LXWP2(line, info);
  else if (type == "$LXWP3"sv)
    return LXWP3(line, info);
  else
    return false;
}

bool
LXEosDevice::SendNewSettings(OperationEnvironment& env)
{
  if (vario_settings.uptodate == true) {
    char buffer[64];
    snprintf(buffer,
             64,
             "PFLX2,%.1f,%.2f,%.0f,,,,,",
             vario_settings.mc,
             vario_settings.bal,
             vario_settings.bugs);

    PortWriteNMEA(port, buffer, env);
    return true;
  } else
    return false;
}

bool
LXEosDevice::WriteAndWaitForACK(const std::span<std::byte>& message,
                                OperationEnvironment& env)
{
  port.FullFlush(
    env, std::chrono::milliseconds(50), std::chrono::milliseconds(200));
  port.FullWrite(message, env, std::chrono::milliseconds(1000));

  try {
    port.WaitForByte(
      // ACK is 0x06 (NACK is 0x15)
      std::byte{ 0x06 },
      env,
      std::chrono::milliseconds(3000));
  } catch (...) {
    return false;
  }
  return true;
}

void
LXEosDevice::CopyStringSpacePadded(char dest[],
                                   const TCHAR src[],
                                   const uint8_t len)
{
  bool src_end_reached = false;
  for (uint8_t i = 0; i < (len - 1); i++) {
    if (!src_end_reached)
      if (src[i] == 0)
        src_end_reached = true;
    dest[i] = src_end_reached ? '\x20' : src[i];
  }

  dest[len - 1] = 0;
}

uint32_t
LXEosDevice::ConvertCoord(Angle coord)
{
  int32_t valueMilliMinutes = static_cast<int32_t>(coord.Degrees() * 60000.0);
  return ToBE32(*reinterpret_cast<uint32_t*>(&valueMilliMinutes));
}

uint8_t
LXEosDevice::CalculateCRC(std::byte* msg, const int len, const uint8_t initial)
{
  uint8_t result{ initial };

  for (int byte = 0; byte < len; byte++) {
    uint8_t d = static_cast<uint8_t>(msg[byte]);
    for (int count = 8; --count >= 0; d <<= 1) {
      uint8_t tmp = result ^ d;
      result <<= 1;
      if ((tmp & 0x80) != 0)
        result ^= 0x69;
    }
  }

  return result;
}
