// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Device/Declaration.hpp"
#include "Device/Driver.hpp"
#include "Device/Driver/LX_Eos.hpp"
#include "Device/Port/Port.hpp"
#include "Device/SettingsMap.hpp"
#include "NMEA/DeviceInfo.hpp"
#include "NMEA/InputLine.hpp"
#include "thread/Mutex.hxx"

using std::string_view_literals::operator""sv;

struct EosDeclarationStruct
{
  const uint8_t syn = 0x02;
  const uint8_t cmd = 0xCA;
  const uint8_t flag = 0;   // Not used
  const uint16_t oo_id = 0; // Not used
  char pilot[19];           // "Name Surname"
  char glider[12];          // Polar name
  char reg_num[8];          // Glider registration number
  char cmp_num[4];          // Competition id

  // 0=STANDARD, 1=15-METER, 2=OPEN, 3=18-METER,
  // 4=WORLD, 5=DOUBLE, 6=MOTOR_GL
  uint8_t byClass;
  char observer[10];              // Not used
  const uint8_t gpsdatum = 0;     // Not used
  const uint8_t fix_accuracy = 0; // Not used
  char gps[60];                   // Not used

  /* auto defined */
  const uint8_t flag2 = 0;      // Not used
  const int32_t input_time = 0; // Not used
  const uint8_t di = 0;         // Not used
  const uint8_t mi = 0;         // Not used
  const uint8_t yi = 0;         // Not used
  /* user defined */
  const uint8_t fd = 0;     // Not used
  const uint8_t fm = 0;     // Not used
  const uint8_t fy = 0;     // Not used
  const int16_t taskid = 0; // Not used

  // Number of TP without Takeoff, Start, Finish and Landing.
  char num_of_tp;

  // 1=Turnpoint (also Start and Finish), 2=Landing, 3=Takeoff
  uint8_t prg[12];

  // TP Longitude in degrees multiplied by 60000.0f
  int32_t lon[12];

  //< TP Latitude in degrees multiplied by 60000.0f
  int32_t lat[12];

  char name[12][9]; //< TP Name

  uint8_t crc;
} gcc_packed;

struct EosObsZoneStruct
{
  const uint8_t syn = 0x02;
  const uint8_t cmd = 0xF4;

  // TP number 1=Start, 2 = TP1, 3=TP2, 4=Finish
  uint8_t uiTpNr;

  // direction 0= Symmetric, 1=Fixed, 2=Next, 3=Previous
  uint8_t uiDirection;

  uint8_t bAutoNext = 1; // Is this auto next TP or AAT TP
  uint8_t bIsLine = 0;   // Is this line flag
  float fA1;             // Angle A1 in radians
  float fA2 = 0;         // Angle A2 in radians
  float fA21 = 0;        // Angle A21 in radians
  uint32_t uiR1;         // Radius R1 in meters
  uint32_t uiR2 = 0;     // Radius R2 in meters
  float fElevation;      // Turnpoint elevation

  uint8_t crc;
} gcc_packed;

struct EosClassStruct
{
  const uint8_t syn = 0x02;
  const uint8_t cmd = 0xD0;
  char name[9];     // Competition class
  uint8_t crc;
} gcc_packed;

/**
 * @brief Struct to hold last known settings of the device
 *
 */
struct VarioSettings
{
  float mc = 0;          // Mc Cready in m/s
  float bugs = 0;        // Bugs in percent (lower value = less bugs)
  float bal = 1;         // Glider mass divided by polar reference mass
  bool uptodate = false; // Setting were received from device at least once
};

class LXEosDevice : public AbstractDevice
{
  Port& port;

public:
  explicit LXEosDevice(Port& _port)
    : port(_port)
  {
  }

private:
  VarioSettings vario_settings; // last known settings of the device
  Mutex settings_mutex;

  bool LXWP0(NMEAInputLine& line, NMEAInfo& info);
  bool LXWP1(NMEAInputLine& line, DeviceInfo& device);
  bool LXWP2(NMEAInputLine& line, NMEAInfo& info);
  bool LXWP3(NMEAInputLine& line, NMEAInfo& info);

  /**
   * @brief Sends settings from vario_settings struct to device
   *
   * @param env
   * @return true if successful
   * @return false if vario_settings are not uptodate (previous settings are
   * unknown)
   */
  bool SendNewSettings(OperationEnvironment& env);

  /**
   * @brief Calculate one byte CRC checksum using 0x69 polynomial
   *
   * @param msg crc is calculated using all bytes of this array
   * @param len length of msg
   * @param initial initial value of byte (typically 0xFF, but may differ)
   * @return uint8_t CRC byte
   */
  static uint8_t CalculateCRC(std::byte* msg, const int len, const uint8_t initial);

  /**
   * @brief Fills destination buffer of given length by characters of string
   * followed by spaces and ended by 0x00
   *
   * @param dest destination buffer
   * @param src source buffer (0x00 terminated string)
   * @param len length of destination buffer
   */
  static void CopyStringSpacePadded(char dest[],
                                    const TCHAR src[],
                                    const uint8_t len);

  /**
   * @brief Converts coordinate to value used in declaration
   *
   * @param coord coordinate (longitude of latitude)
   * @return uint32_t variable representing 32bit signed integer in big-endian
   * @note value is in milli angle minutes (x60000)
   */
  static uint32_t ConvertCoord(Angle coord);

  /**
   * @brief Send declaration (pilot, glider, and waypoints) to Eos
   *
   * @param declaration
   * @param home (unused)
   * @param env
   * @return true if WriteAndWaitForACK was successful,
   * @return false if unsuccessful or if there is invalid number of waypoints
   */
  bool SendDeclaration(const Declaration& declaration,
                       const Waypoint* home,
                       OperationEnvironment& env);

  /**
   * @brief
   *
   * @param tp_nr turnpoint number (counted from 1 = Start)
   * @param declaration
   * @param env
   * @return true if WriteAndWaitForACK was successful,
   * @return false if unsuccessful or if the tp_nr is invalid
   */
  bool SendObsZone(uint8_t tp_nr,
                   const Declaration& declaration,
                   OperationEnvironment& env);

  /**
   * @brief Send competition class string (currently an empty string, as there
   * is no class info in the XCSoar declaration)
   *
   * @param declaration
   * @param env
   * @return true
   * @return false
   */
  bool SendCompetitionClass(const Declaration& declaration,
                            OperationEnvironment& env);
  /**
   * @brief Transmit data and wait for ACK response
   *
   * @param message Data to be transmitted
   * @param env OperationEnvironment
   * @return true if ACK was received,
   * @return false if time out or other exception occurred
   * @note timeout is 1 second
   */
  bool WriteAndWaitForACK(const std::span<std::byte>& message,
                          OperationEnvironment& env);

public:
  /* virtual methods from class Device */
  void LinkTimeout() override;
  bool EnableNMEA(OperationEnvironment& env) override;
  bool ParseNMEA(const char* line, struct NMEAInfo& info) override;
  bool PutBugs(double bugs, OperationEnvironment& env) override;
  bool PutMacCready(double mc, OperationEnvironment& env) override;
  bool Declare(const Declaration& declaration,
               const Waypoint* home,
               OperationEnvironment& env) override;
};
