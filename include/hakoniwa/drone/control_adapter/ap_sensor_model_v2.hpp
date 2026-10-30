// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <array>
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"

namespace hakoniwa::drone::control_adapter {
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
std::array<double, 3> ap_rotate_sensor_to_body(
    const ArdupilotSensorModelV2Config&, const std::array<double, 3>&);
std::array<double, 3> ap_correct_mag_sensor_to_body(
    const ArdupilotSensorModelV2Config&, const std::array<double, 3>& sensor_gauss);
#endif
}
