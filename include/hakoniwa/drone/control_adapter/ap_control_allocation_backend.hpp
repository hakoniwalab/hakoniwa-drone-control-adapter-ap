// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"

#include <memory>

namespace hakoniwa::drone::control_adapter {

struct ApControlAllocationBackendConfig {
    double throttle_rpy_mix{0.5};
    double yaw_headroom{0.2};
};

class ApControlAllocationBackend final : public IControlAllocationBackend {
public:
    explicit ApControlAllocationBackend(
        const ApControlAllocationBackendConfig& config = {});

    void reset() override;
    ControlAllocationOutput run(const ControlAllocationInput& input) override;

    void set_config(const ApControlAllocationBackendConfig& config);
    // Where run() publishes AP_MotorsMatrix's limit.throttle_lower/upper for the altitude stages.
    void set_motor_limits_sink(std::shared_ptr<ApMotorThrottleLimits> sink) { motor_limits_ = std::move(sink); }

private:
    ApControlAllocationBackendConfig config_{};
    std::shared_ptr<ApMotorThrottleLimits> motor_limits_{};
};

}  // namespace hakoniwa::drone::control_adapter
