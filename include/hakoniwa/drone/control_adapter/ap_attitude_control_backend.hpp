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
};

}  // namespace hakoniwa::drone::control_adapter
