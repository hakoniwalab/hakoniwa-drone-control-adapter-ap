#pragma once

#include <string>
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
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
struct ArdupilotControllerConfig {
    ArdupilotControllerRuntimeConfig runtime{};
    ArdupilotSensorFilterConfig sensor_filter{};
    ApAltitudeControlBackendConfig altitude_control{};
    ApHorizontalPositionControlBackendConfig horizontal_control{};
    ApAttitudeControlBackendConfig attitude_control{};
    ApRateControlBackendConfig rate_control{};
    ApControlAllocationBackendConfig control_allocation{};
};
class ArdupilotControllerConfigLoader {
public:
    ArdupilotControllerConfig load_from_file(const std::string&) const;
    ArdupilotControllerConfig load_from_text(const std::string&) const;
};
}  // namespace hakoniwa::drone::control_adapter
