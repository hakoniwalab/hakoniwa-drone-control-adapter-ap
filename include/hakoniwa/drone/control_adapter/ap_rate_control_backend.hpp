#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"

#include <memory>

namespace hakoniwa::drone::control_adapter {

struct ApRateControlAxisConfig {
    double p{0.0};
    double i{0.0};
    double d{0.0};
    double feed_forward{0.0};
    double integrator_limit{0.0};
    double target_filter_hz{0.0};
    double error_filter_hz{0.0};
    double derivative_filter_hz{0.0};
    double slew_rate_max{0.0};
    double slew_rate_tau{1.0};
    double derivative_feed_forward{0.0};
};

struct ApRateControlBackendConfig {
    ApRateControlAxisConfig roll{};
    ApRateControlAxisConfig pitch{};
    ApRateControlAxisConfig yaw{};
};

struct ApRateControlAxisStatus {
    double proportional{0.0};
    double integral{0.0};
    double derivative{0.0};
    double feed_forward{0.0};
};

struct ApRateControlBackendStatus {
    ApRateControlAxisStatus roll{};
    ApRateControlAxisStatus pitch{};
    ApRateControlAxisStatus yaw{};
};

class ApRateControlBackend final : public IRateControlBackend {
public:
    explicit ApRateControlBackend(const ApRateControlBackendConfig& config);
    ~ApRateControlBackend() override;

    ApRateControlBackend(const ApRateControlBackend&) = delete;
    ApRateControlBackend& operator=(const ApRateControlBackend&) = delete;

    void reset() override;
    BodyTorqueCommand run(const RateControlInput& input) override;

    void set_config(const ApRateControlBackendConfig& config);
    ApRateControlBackendStatus get_status() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace hakoniwa::drone::control_adapter
