// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

namespace hakoniwa::drone::control_adapter {

/**
 * AC_PosControl::input_pos_xyz: a position target is not handed to the position P controller as it
 * is. ArduPilot moves a desired position toward it along a kinematic path limited in speed,
 * acceleration and jerk, and controls to that path, with its velocity and acceleration as
 * feed-forward (AC_PosControl _pos_desired, _vel_desired, _accel_desired). Guided mode sets the
 * limits to the commanded or default speed (WPNAV_SPEED, WPNAV_SPEED_UP/DN) and WPNAV_ACCEL /
 * WPNAV_ACCEL_Z, and the jerk to PSC_JERK_XY / PSC_JERK_Z. Speeds come from the stages' limits.
 * enabled false hands the target straight to the stages (the interface contract test checks one
 * call's response that way).
 */
struct ApPositionShapingConfig {
    bool enabled{true};
    double acceleration_xy_mps2{2.5};   // WPNAV_ACCEL
    double jerk_xy_mps3{5.0};           // PSC_JERK_XY
    double acceleration_z_mps2{2.5};    // WPNAV_ACCEL_Z
    double jerk_z_mps3{5.0};            // PSC_JERK_Z
};

/**
 * ArduPilot's 3D position control: AC_PosControl runs its z and xy controllers
 * together, which here are the altitude (AC_P_1D/AC_PID) and horizontal
 * (AC_P_2D/AC_PID_2D) stages of this adapter. The stage turns their collective
 * thrust and lean angles, with the yaw target, into the target attitude.
 */
class ApPositionControl3DBackend final : public IPositionControl3DBackend {
public:
    ApPositionControl3DBackend(
        const ApAltitudeControlBackendConfig& altitude,
        const ApHorizontalPositionControlBackendConfig& horizontal,
        const ApPositionShapingConfig& shaping = ApPositionShapingConfig{false});

    void reset() override;
    PositionControl3DOutput run_position(const PositionControl3DPositionInput& input, double dt_sec) override;
    PositionControl3DOutput run_velocity(const PositionControl3DVelocityInput& input, double dt_sec) override;
    void set_motor_limits_source(std::shared_ptr<const ApMotorThrottleLimits> source) { altitude_.set_motor_limits_source(std::move(source)); }

private:
    PositionControl3DOutput run(
        const PositionControl3DState& state,
        AltitudeControlInput altitude,
        HorizontalPositionControlInput horizontal,
        const std::optional<double>& target_yaw_rad,
        const std::optional<double>& target_yaw_rate_rad_sec,
        double dt_sec);

    // AC_PosControl's desired path (NE metres, up-positive metres): input_pos_xyz
    void shape_position_target(const PositionControl3DState& state, const Vector3D& target_ned, double dt_sec);

    ApAltitudeControlBackend altitude_;
    ApHorizontalPositionControlBackend horizontal_;
    ApPositionShapingConfig shaping_{};
    bool shaping_started_{false};
    double pos_ne_[2]{0.0, 0.0}, vel_ne_[2]{0.0, 0.0}, accel_ne_[2]{0.0, 0.0};
    double pos_up_{0.0}, vel_up_{0.0}, accel_up_{0.0};
};

}  // namespace hakoniwa::drone::control_adapter
