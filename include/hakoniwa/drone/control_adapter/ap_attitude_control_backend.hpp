#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/attitude_control_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct ApAttitudeControlBackendConfig {
    double angle_roll_p{4.5};
    double angle_pitch_p{4.5};
    double angle_yaw_p{4.5};
    double rate_yaw_p{0.18};
    double accel_roll_max_rad_sec2{0.0};
    double accel_pitch_max_rad_sec2{0.0};
    double accel_yaw_max_rad_sec2{0.0};
    double rate_roll_max_rad_sec{0.0};
    double rate_pitch_max_rad_sec{0.0};
    double rate_yaw_max_rad_sec{0.0};
    bool use_sqrt_controller{true};
    // AC_AttitudeControl input shaping (ATC_RATE_FF_ENAB, default on): the controller tracks
    // an internal target attitude that follows the commanded one through
    // input_shaping_angle() with the time constant ATC_INPUT_TC and the accel limits, and
    // feeds the target's angular velocity forward. Off: the commanded attitude is the target.
    bool rate_feedforward_enabled{true};
    double input_time_constant_sec{0.15};   // ATC_INPUT_TC
    double slew_yaw_rad_sec{6000.0 * 3.14159265358979323846 / 18000.0};  // ATC_SLEW_YAW (cdeg/s)
};

struct ApAttitudeControlBackendStatus {
    double thrust_error_angle_rad{0.0};
    double feedforward_scalar{1.0};
};

class ApAttitudeControlBackend final : public IAttitudeControlBackend {
public:
    explicit ApAttitudeControlBackend(
        const ApAttitudeControlBackendConfig& config = {});

    void reset() override;
    AngularRateTarget run(const AttitudeControlInput& input) override;

    void set_config(const ApAttitudeControlBackendConfig& config);
    ApAttitudeControlBackendStatus get_status() const;

private:
    ApAttitudeControlBackendConfig config_{};
    ApAttitudeControlBackendStatus status_{};
    // Input-shaping state (AC_AttitudeControl _attitude_target, _ang_vel_target)
    bool shaping_started_{false};
    double target_w_{1.0}, target_x_{0.0}, target_y_{0.0}, target_z_{0.0};
    double ang_vel_target_x_{0.0}, ang_vel_target_y_{0.0}, ang_vel_target_z_{0.0};
};

}  // namespace hakoniwa::drone::control_adapter
