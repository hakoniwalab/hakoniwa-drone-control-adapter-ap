#pragma once
// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/horizontal_position_control_backend.hpp"
#include <memory>
namespace hakoniwa::drone::control_adapter {
struct ApHorizontalPositionControlBackendConfig {
    double position_p{1.0}, velocity_p{2.0}, velocity_i{1.0}, velocity_d{0.25}, velocity_feed_forward{0.0};
    double velocity_imax_mps2{10.0}, velocity_error_filter_hz{5.0}, velocity_derivative_filter_hz{5.0};
    double speed_max_mps{5.0}, acceleration_max_mps2{5.0}, jerk_max_mps3{5.0};
    double angle_max_rad{0.7853981633974483};
};
class ApHorizontalPositionControlBackend final : public IHorizontalPositionControlBackend {
public:
    explicit ApHorizontalPositionControlBackend(const ApHorizontalPositionControlBackendConfig& = {});
    ~ApHorizontalPositionControlBackend() override;
    void reset() override;
    HorizontalTiltTarget run(const HorizontalPositionControlInput&, double dt_sec) override;
    void set_config(const ApHorizontalPositionControlBackendConfig&);
    // AC_PosControl _accel_desired.xy (NE m/s^2), added to the velocity PID's acceleration target as
    // update_xy_controller does; set by the 3D stage's input shaping, 0 otherwise.
    void set_acceleration_feedforward(double north_mps2, double east_mps2) { acceleration_feedforward_x_ = north_mps2; acceleration_feedforward_y_ = east_mps2; }
    double speed_max_mps() const { return config_.speed_max_mps; }
private:
    class Impl; ApHorizontalPositionControlBackendConfig config_{}; std::unique_ptr<Impl> impl_;
    double acceleration_target_x_{0.0}, acceleration_target_y_{0.0};
    double acceleration_feedforward_x_{0.0}, acceleration_feedforward_y_{0.0};
};
}
