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
    // AC_PosControl hands its throttle to the motors through a first-order low-pass
    // (set_throttle_out(..., POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ = 2 Hz), applied by
    // AP_Motors::_throttle_filter). 0 = no filter.
    double throttle_filter_hz{2.0};
    // AC_PosControl input shaping of a position (altitude) target: ArduPilot moves a desired altitude
    // toward the target within speed_up/down_mps, acceleration_max_mps2 (WPNAV_ACCEL_Z) and
    // jerk_max_mps3 (PSC_JERK_Z), controls to it and feeds its velocity and acceleration forward
    // (input_pos_xyz / set_pos_target_z_from_climb_rate). The 3D stage shapes its own target and turns
    // this off for the altitude stage it runs. false hands the target straight to the position P.
    bool input_shaping{false};
};
// AP_Motors limit.throttle_lower/upper of the previous cycle, which AC_PosControl::update_z_controller
// passes to the vertical velocity and acceleration PIDs. The plugin shares one between its allocation
// (writer) and its altitude stages (readers); a stage without one sees no limit.
struct ApMotorThrottleLimits {
    bool lower{false};
    bool upper{false};
};
class ApAltitudeControlBackend final : public IAltitudeControlBackend {
public:
    explicit ApAltitudeControlBackend(const ApAltitudeControlBackendConfig& = {});
    ~ApAltitudeControlBackend() override;
    void reset() override;
    NormalizedVerticalThrustCommand run(const AltitudeControlInput&, double dt_sec) override;
    void set_config(const ApAltitudeControlBackendConfig&);
    void set_motor_limits_source(std::shared_ptr<const ApMotorThrottleLimits> source) { motor_limits_ = std::move(source); }
    // The target thrust angle (rad from vertical) of the attitude target AC_AttitudeControl built on
    // the previous cycle, which get_throttle_boosted divides by. Without one the current tilt is used.
    void set_target_thrust_angle(double angle_rad) { target_thrust_angle_rad_ = angle_rad; has_target_thrust_angle_ = true; }
    // AC_PosControl _accel_desired.z (up-positive m/s^2), added to the velocity PID's acceleration
    // target as update_z_controller does; set by the 3D stage's input shaping, 0 otherwise.
    void set_acceleration_feedforward(double up_mps2) { acceleration_feedforward_mps2_ = up_mps2; }
    double speed_up_mps() const { return config_.speed_up_mps; }
    void set_input_shaping(bool enabled) { config_.input_shaping = enabled; shaping_started_ = false; }
    double speed_down_mps() const { return config_.speed_down_mps; }
private:
    class Impl; ApAltitudeControlBackendConfig config_{}; std::unique_ptr<Impl> impl_;
    double acceleration_target_mps2_{0.0};
    double acceleration_feedforward_mps2_{0.0};
    bool shaping_started_{false};
    double shaped_pos_up_{0.0}, shaped_vel_up_{0.0}, shaped_accel_up_{0.0};
    double throttle_filtered_{0.0};
    bool throttle_filter_started_{false};
    std::shared_ptr<const ApMotorThrottleLimits> motor_limits_{};
    double target_thrust_angle_rad_{0.0};
    bool has_target_thrust_angle_{false};
};
}
