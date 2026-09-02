// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "hakoniwa/drone/control_adapter/control_allocation_backend.hpp"

namespace hakoniwa::drone::control_adapter {

struct ApControlAllocationBackendConfig {
    double hover_thrust{0.35};
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

private:
    ApControlAllocationBackendConfig config_{};
};

}  // namespace hakoniwa::drone::control_adapter
