// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace hakoniwa::drone::control_adapter {
// Geometric altitude (m) for a pressure (Pa) in the 1976 standard atmosphere, as ArduPilot's
// AP_Baro::get_altitude_from_pressure() (the formula SITL builds use). NaN for a non-positive pressure.
float ap_altitude_from_pressure_1976(float pressure_pa);
}
