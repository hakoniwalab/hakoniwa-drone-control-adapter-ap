// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include "hakoniwa/drone/control_adapter/ekf_adapter.hpp"

namespace hakoniwa::drone::control_adapter {
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
std::array<double, 3> ap_rotate_sensor_to_body(
    const ArdupilotSensorModelV2Config&, const std::array<double, 3>&);
std::array<double, 3> ap_correct_mag_sensor_to_body(
    const ArdupilotSensorModelV2Config&, const std::array<double, 3>& sensor_gauss);
// [SEN-GPS-380] The fix as AP_GPS reads it from ArduPilot SITL's u-blox UBX-NAV-PVT: SITL writes lat/lon in 1e-7 deg,
// height in mm, velocity in mm/s and accuracies in mm (casts truncate toward zero); AP_GPS_UBLOX keeps lat/lon,
// takes the altitude as h_msl / 10 cm and scales the rest back by 1e-3. The speed accuracy is written in mm/s
// (SITL writes cm/s into that field, which reads back as a tenth; that is not reproduced).
EkfHilGpsInput ap_ublox_pvt_fix(const EkfHilGpsInput& input);
// [SEN-ACC-300] As AP_InertialSensor_Backend::_rotate_and_correct_accel(): subtract INS_ACCOFFS, multiply by
// INS_ACCSCAL in the sensor axes, then rotate to the body axes.
std::array<double, 3> ap_correct_accel_sensor_to_body(
    const ArdupilotSensorModelV2Config&, const std::array<double, 3>& sensor_mps2);
#endif
}
