// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Geo/SpeedVector.hpp"
#include "LXEosDevice.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/System.hpp"
#include "util/Macros.hpp"
#include "util/StringCompare.hxx"

bool
LXEosDevice::LXWP0(NMEAInputLine& line, NMEAInfo& info)
{
  /*
   * $LXWP0,Y,119.4,1717.6,0.02,0.02,0.02,0.02,0.02,0.02,,000,107.2*5b
   *
   * <is_logger_running> char     'Y'=yes, 'N'=no
   * <tas>               float    True airspeed in km/h
   * <altitude>          float    True altitude in meters
   * <varioN>            float    6 measurements of vario in last second in m/s
   * <heading>           uint16_t True heading in deg. Blank if no compass.
   * <wind_direction>    string   Wind dir in deg. Blank if spd is 0.0.
   * <wind_speed>        string   Wind speed in km/h. Blank if wind speed is 0.
   */

  line.Skip();

  double airspeed;
  bool tas_ok = line.ReadChecked(airspeed);
  if (tas_ok && (airspeed < -50 || airspeed > 400))
    /* implausible */
    return false;

  double altitude_true;
  if (line.ReadChecked(altitude_true))
    info.ProvideBaroAltitudeTrue(altitude_true);

  if (tas_ok)
    /*
     * Call ProvideTrueAirspeed() after ProvideBaroAltitudeTrue() to use
     * the provided altitude (if available)
     */
    info.ProvideTrueAirspeed(
      Units::ToSysUnit(airspeed, Unit::KILOMETER_PER_HOUR));

  double vario = 0;
  bool vario_ok = true;
  double value = 0;
  // Filter the 6 reading using 5th order low-pass FIR filter
  for (double fir_b : { -0.0421, 0.1628, 0.3793, 0.3793, 0.1628, -0.0421 }) {
    vario_ok = vario_ok && line.ReadChecked(value);
    vario += value * fir_b;
  }

  if (vario_ok)
    info.ProvideTotalEnergyVario(vario);

  line.Skip(1); // Heading
  line.Skip(1); // Eos seems to put one more empty value that is not documented

  bool wind_ok = true;
  double wind_direction, wind_speed;
  wind_ok = line.ReadChecked(wind_direction);
  wind_ok = wind_ok && line.ReadChecked(wind_speed);
  SpeedVector wind;
  if (wind_ok) {
    wind.bearing = Angle::Degrees(wind_direction);
    wind.norm = Units::ToSysUnit(wind_speed, Unit::KILOMETER_PER_HOUR);
    info.ProvideExternalWind(wind);
  }

  return true;
}

bool
LXEosDevice::LXWP1(NMEAInputLine& line, DeviceInfo& device)
{
  /*
   * $LXWP1,
   * <device_name> string   LX device name
   * <serial>      uint32_t serial number
   * <sw_version>  float    firmware version
   * <hw_version>  float    hardware version
   */

  device.product = line.ReadView();
  device.serial = line.ReadView();
  device.software_version = line.ReadView();
  device.hardware_version = line.ReadView();

  return true;
}

bool
LXEosDevice::LXWP2(NMEAInputLine& line, NMEAInfo& info)
{
  /*
   * $LXWP2,
   * <mc>          float    MacCready factor/s
   * <load_factor> float    Total glider mass divided by polar reference mass
   * <bugs>        uint16_t Bugs factor in percent
   * <polar_a>     float    Polar -square coefficient, velocity in m/s
   * <polar_b>     float    Polar -linear coefficient, velocity in m/s
   * <polar_c>     float    Polar -constant coefficient, velocity in m/s
   * <volume>      uint8_t  Variometer volume in percent
   */

  /*
   * Ballast is expressed as Total glider mass divided by polar reference mass.
   * Apart from water ballast, it also includes pilot weight as set in vario).
   * Reference mass of polar in vario may differ from XCSoar.
   * There is no straightforward way to get a useful value from it. Not used.
   */

  double mc, bal, bugs;

  if (!line.ReadChecked(mc))
    return false;
  if (!line.ReadChecked(bal))
    return false;
  if (!line.ReadChecked(bugs))
    return false;
  line.Skip(3); // Polar coefficients
  line.Skip(1); // Vario volume

  /*
   * Sending setting from XCSoar to vario requires sending MC, Bugs, and
   * Ballast all in one sentence. It is not possible to set one without
   * affecting the others. Last received settings are stored in vario_settings
   * struct. These will be used to set the other values when changing one of
   * them.
   */
  const std::lock_guard lock{ settings_mutex };

  info.settings.ProvideMacCready(mc, info.clock);
  double bugs_xcsoar = static_cast<double>(100 - bugs) / 100.0;
  info.settings.ProvideBugs(bugs_xcsoar, info.clock);

  vario_settings.mc = mc;
  vario_settings.bal = bal; // The original value (in percent) is stored
  vario_settings.bugs = bugs;
  vario_settings.uptodate = true;

  return true;
}

bool
LXEosDevice::LXWP3(NMEAInputLine& line, NMEAInfo& info)
{
  /*
   * $LXWP3,
   * <sc_mode>     uint8_t  SC mode. 0 = manual, 1 = circling, 2 = speed
   * <filter>      float    SC filter factor in seconds
   * <reserved>    Reserved
   * <te_level>    uint16_t TE level in percent
   * <int_time>    uint16_t SC integration time in seconds
   * <range>       uint8_t  SC range in m/s
   * <silence>     float    SC silence in m/s
   * <switch_mode> uint8_t  SC switch mode. 0 = off, 1 = on, 2 = toggle.
   * <speed>       uint16_t SC speed in km/h
   * <polar_name>  string   Self explanatory
   * <reserved>    Reserved
   */

  double value;

  // Altitude offset -> QNH
  if (line.ReadChecked(value)) {
    value = Units::ToSysUnit(-value, Unit::FEET);
    auto qnh = AtmosphericPressure::PressureAltitudeToStaticPressure(value);
    info.settings.ProvideQNH(qnh, info.clock);
  }

  // No other are used

  return true;
}
