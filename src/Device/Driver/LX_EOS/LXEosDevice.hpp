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

public:
  /* virtual methods from class Device */
  void LinkTimeout() override;
  bool EnableNMEA(OperationEnvironment& env) override;
  bool ParseNMEA(const char* line, struct NMEAInfo& info) override;
  bool PutBugs(double bugs, OperationEnvironment& env) override;
  bool PutMacCready(double mc, OperationEnvironment& env) override;
};
