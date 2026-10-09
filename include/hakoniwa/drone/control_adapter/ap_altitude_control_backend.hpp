#pragma once
// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/altitude_control_backend.hpp"
#include <memory>
namespace hakoniwa::drone::control_adapter {
struct ApAltitudeControlBackendConfig {
    double position_p{1.0}, velocity_p{5.0}, velocity_i{0.0}, velocity_d{0.0};
    double velocity_imax_mps2{10.0}, velocity_filter_hz{5.0}, velocity_derivative_filter_hz{5.0}, velocity_feed_forward{0.0};
    double acceleration_p{0.5}, acceleration_i{1.0}, acceleration_d{0.0}, acceleration_imax{0.8};
    double acceleration_target_filter_hz{0.0}, acceleration_error_filter_hz{20.0}, acceleration_derivative_filter_hz{20.0}, acceleration_feed_forward{0.0};
    double hover_thrust{0.35}, speed_up_mps{2.5}, speed_down_mps{1.5};
    double acceleration_max_mps2{2.5}, jerk_max_mps3{5.0}, thrust_min{0.0}, thrust_max{1.0};
};
class ApAltitudeControlBackend final : public IAltitudeControlBackend {
public:
    explicit ApAltitudeControlBackend(const ApAltitudeControlBackendConfig& = {});
    ~ApAltitudeControlBackend() override;
    void reset() override;
    NormalizedVerticalThrustCommand run(const AltitudeControlInput&, double dt_sec) override;
    void set_config(const ApAltitudeControlBackendConfig&);
private:
    class Impl; ApAltitudeControlBackendConfig config_{}; std::unique_ptr<Impl> impl_;
    double acceleration_target_mps2_{0.0};
};
}
