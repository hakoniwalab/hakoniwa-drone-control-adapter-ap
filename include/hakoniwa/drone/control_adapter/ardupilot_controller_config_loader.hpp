// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <string>
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_rate_control_backend.hpp"

namespace hakoniwa::drone::control_adapter {
struct ArdupilotControllerRuntimeConfig {
    double altitude_hz{50.0}, horizontal_hz{50.0}, attitude_hz{100.0}, rate_hz{400.0};
};
// INS_GYRO_FILTER / INS_ACCEL_FILTER: ArduPilot's second-order low-pass on the
// gyro (rate control input) and the accelerometer (vertical acceleration of the
// altitude and 3D position control). 0 = no filter, as AP_InertialSensor.
struct ArdupilotSensorFilterConfig {
    double gyro_cutoff_hz{0.0}, accel_cutoff_hz{0.0};
};
// The sensor delays NavEKF3 allows for (GPS1_DELAY_MS, EK3_HGT_DELAY): set them to the
// vehicle's real sensor delays (Drone PRO components.sensors.gps/baro.delayMsec) so the EKF
// fuses each measurement at its own time. GPS1_DELAY_MS 0 means "no delay" here (ArduPilot
// takes 0 as "the driver's default"; the exporter writes at least 1). The in-process
// NavEKF3 keeps its frontend parameters at their defaults, so EK3_HGT_DELAY must be 60.
struct ArdupilotEkfConfig {
    double gps_delay_ms{0.0};
    double hgt_delay_ms{60.0};
};
struct ArdupilotControllerConfig {
    ArdupilotControllerRuntimeConfig runtime{};
    ArdupilotSensorFilterConfig sensor_filter{};
    ArdupilotEkfConfig ekf{};
    // AC_PosControl input shaping of the 3D stage's position target (WPNAV_ACCEL, WPNAV_ACCEL_Z in m/s^2,
    // PSC_JERK_XY, PSC_JERK_Z in m/s^3; runtime.position_input_shaping 0 turns it off). An acceleration_xy
    // of 0 is derived when the stage is made: 0.5 g tan(lean angle limit), ArduCopter's WPNAV_ACCEL
    // as the exporter writes it.
    ApPositionShapingConfig position_shaping{true, 0.0, 5.0, 2.5, 5.0};
    ApAltitudeControlBackendConfig altitude_control{};
    ApHorizontalPositionControlBackendConfig horizontal_control{};
    ApAttitudeControlBackendConfig attitude_control{};
    ApRateControlBackendConfig rate_control{};
    ApControlAllocationBackendConfig control_allocation{};
};
// The 3D stage's shaping limits with WPNAV_ACCEL derived from the lean limit when not given.
inline ApPositionShapingConfig position_shaping_for(const ArdupilotControllerConfig& c)
{
    ApPositionShapingConfig shaping = c.position_shaping;
    if (shaping.enabled && !(shaping.acceleration_xy_mps2 > 0.0)) {
        shaping.acceleration_xy_mps2 = 0.5 * 9.80665 * std::tan(c.horizontal_control.angle_max_rad);
    }
    return shaping;
}
class ArdupilotControllerConfigLoader {
public:
    ArdupilotControllerConfig load_from_file(const std::string&) const;
    ArdupilotControllerConfig load_from_text(const std::string&) const;
};
}  // namespace hakoniwa::drone::control_adapter
