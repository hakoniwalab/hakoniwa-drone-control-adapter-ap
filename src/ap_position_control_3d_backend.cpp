// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"

#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {

namespace {

// Body (FRD) velocity from the NED velocity and the body-to-NED attitude.
Vector3D ned_to_body(const AttitudeQuaternion& q, const Vector3D& v)
{
    // Rotate by the conjugate of q (NED -> body).
    const double w = q.w, x = -q.x, y = -q.y, z = -q.z;
    const double tx = 2.0 * (y * v.z - z * v.y);
    const double ty = 2.0 * (z * v.x - x * v.z);
    const double tz = 2.0 * (x * v.y - y * v.x);
    return {v.x + w * tx + (y * tz - z * ty), v.y + w * ty + (z * tx - x * tz), v.z + w * tz + (x * ty - y * tx)};
}

AttitudeQuaternion from_euler(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy};
}

}  // namespace

ApPositionControl3DBackend::ApPositionControl3DBackend(
    const ApAltitudeControlBackendConfig& altitude,
    const ApHorizontalPositionControlBackendConfig& horizontal,
    const ApPositionShapingConfig& shaping)
    : altitude_(altitude), horizontal_(horizontal), shaping_(shaping)
{
    altitude_.set_input_shaping(false);  // this stage shapes the 3D target itself
    const double limits[]{shaping.acceleration_xy_mps2, shaping.jerk_xy_mps3, shaping.acceleration_z_mps2, shaping.jerk_z_mps3};
    for (double limit : limits) {
        if (shaping.enabled && !(std::isfinite(limit) && limit > 0.0)) {
            throw std::invalid_argument("ArduPilot position shaping limits must be positive");
        }
    }
}

void ApPositionControl3DBackend::reset()
{
    altitude_.reset();
    horizontal_.reset();
    altitude_.set_acceleration_feedforward(0.0);
    horizontal_.set_acceleration_feedforward(0.0, 0.0);
    shaping_started_ = false;
}

// AC_PosControl::input_pos_xyz (and init_xy_controller / init_z_controller for the first call):
// advance the desired path by dt with its acceleration, then shape a new jerk-limited acceleration
// toward the target, the speed split between xy and z along the direction to it (kinematic_limit).
void ApPositionControl3DBackend::shape_position_target(
    const PositionControl3DState& state, const Vector3D& target_ned, double dt_sec)
{
    if (!shaping_started_) {
        pos_ne_[0] = state.position.x; pos_ne_[1] = state.position.y;
        vel_ne_[0] = state.velocity.x; vel_ne_[1] = state.velocity.y;
        accel_ne_[0] = accel_ne_[1] = 0.0;
        pos_up_ = -state.position.z; vel_up_ = -state.velocity.z; accel_up_ = 0.0;
        shaping_started_ = true;
    }
    const float dt = static_cast<float>(dt_sec);

    Vector2p pos_xy{pos_ne_[0], pos_ne_[1]};
    Vector2f vel_xy{float(vel_ne_[0]), float(vel_ne_[1])};
    Vector2f accel_xy{float(accel_ne_[0]), float(accel_ne_[1])};
    update_pos_vel_accel_xy(pos_xy, vel_xy, accel_xy, dt, Vector2f{}, Vector2f{}, Vector2f{});
    postype_t pos_z = pos_up_;
    float vel_z = float(vel_up_);
    float accel_z = float(accel_up_);
    update_pos_vel_accel(pos_z, vel_z, accel_z, dt, 0.0f, 0.0f, 0.0f);

    // NEU, as AC_PosControl
    const postype_t target_up = -target_ned.z;
    Vector3f dest{float(target_ned.x - pos_xy.x), float(target_ned.y - pos_xy.y), float(target_up - pos_z)};
    float vel_max_xy = 0.0f;
    float vel_max_z = 0.0f;
    if (is_positive(dest.length_squared())) {
        dest.normalize();
        const float vel_max = kinematic_limit(dest, float(horizontal_.speed_max_mps()),
                                              float(altitude_.speed_up_mps()), float(altitude_.speed_down_mps()));
        vel_max_xy = vel_max * dest.xy().length();
        vel_max_z = fabsf(vel_max * dest.z);
    }
    shape_pos_vel_accel_xy(Vector2p{target_ned.x, target_ned.y}, Vector2f{}, Vector2f{}, pos_xy, vel_xy, accel_xy,
                           vel_max_xy, float(shaping_.acceleration_xy_mps2), float(shaping_.jerk_xy_mps3), dt, false);
    const float accel_max_z = float(shaping_.acceleration_z_mps2);
    shape_pos_vel_accel(target_up, 0.0f, 0.0f, pos_z, vel_z, accel_z, -vel_max_z, vel_max_z,
                        -constrain_float(accel_max_z, 0.0f, 7.5f), accel_max_z, float(shaping_.jerk_z_mps3), dt, false);

    pos_ne_[0] = pos_xy.x; pos_ne_[1] = pos_xy.y;
    vel_ne_[0] = vel_xy.x; vel_ne_[1] = vel_xy.y;
    accel_ne_[0] = accel_xy.x; accel_ne_[1] = accel_xy.y;
    pos_up_ = pos_z; vel_up_ = vel_z; accel_up_ = accel_z;
}

PositionControl3DOutput ApPositionControl3DBackend::run(
    const PositionControl3DState& state,
    AltitudeControlInput altitude,
    HorizontalPositionControlInput horizontal,
    const std::optional<double>& target_yaw_rad,
    const std::optional<double>& target_yaw_rate_rad_sec,
    double dt_sec)
{
    // The altitude stage is up-positive (interface-spec AltitudeControl), the
    // 3D stage NED.
    altitude.attitude = state.attitude;
    altitude.position.z = -state.position.z;
    altitude.velocity.vz = -state.velocity.z;
    altitude.acceleration.az = -state.acceleration.z;
    const Vector3D body = ned_to_body(state.attitude, state.velocity);
    altitude.body_velocity = {body.x, body.y, -body.z};

    horizontal.position = {state.position.x, state.position.y};
    horizontal.velocity = {state.velocity.x, state.velocity.y};
    horizontal.acceleration = {state.acceleration.x, state.acceleration.y};
    // Without a yaw target the heading is held. The lean angles are computed
    // in the heading the target attitude has, so that the attitude's thrust
    // vector points where the horizontal stage asks while the vehicle turns
    // (lean angles in the current heading, applied at the target heading,
    // tilt the wrong way by the heading difference).
    const double yaw = target_yaw_rad.value_or(state.yaw_rad);
    horizontal.yaw_rad = yaw;

    const NormalizedVerticalThrustCommand thrust = altitude_.run(altitude, dt_sec);
    const HorizontalTiltTarget tilt = horizontal_.run(horizontal, dt_sec);

    PositionControl3DOutput output{};
    output.target_attitude = from_euler(tilt.roll_rad, tilt.pitch_rad, yaw);
    // The attitude stage tracks this target from the next cycle; AC_PosControl's throttle boost on
    // the next cycle uses its thrust angle (AC_AttitudeControl _thrust_angle).
    altitude_.set_target_thrust_angle(std::acos(std::clamp(std::cos(tilt.roll_rad) * std::cos(tilt.pitch_rad), -1.0, 1.0)));
    output.thrust = thrust;
    output.target_yaw_rate_rad_sec = target_yaw_rate_rad_sec;
    return output;
}

PositionControl3DOutput ApPositionControl3DBackend::run_position(
    const PositionControl3DPositionInput& input, double dt_sec)
{
    if (shaping_.enabled) {
        if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
            throw std::invalid_argument("position control dt must be positive");
        }
        // Control to the shaped path: its position as the target, its velocity and acceleration as
        // feed-forward (on top of any the caller gives), as AC_PosControl update_xy/z_controller.
        shape_position_target(input.state, input.target_position, dt_sec);
        PositionControl3DPositionInput shaped = input;
        shaped.target_position = {pos_ne_[0], pos_ne_[1], -pos_up_};
        const Vector3D given = input.feedforward_velocity.value_or(Vector3D{});
        shaped.feedforward_velocity = Vector3D{given.x + vel_ne_[0], given.y + vel_ne_[1], given.z - vel_up_};
        horizontal_.set_acceleration_feedforward(accel_ne_[0], accel_ne_[1]);
        altitude_.set_acceleration_feedforward(accel_up_);
        AltitudeControlInput altitude{};
        altitude.mode = AltitudeControlMode::Position;
        altitude.target_altitude = -shaped.target_position.z;
        altitude.target_velocity.vz = -shaped.feedforward_velocity->z;
        HorizontalPositionControlInput horizontal{};
        horizontal.mode = HorizontalControlMode::Position;
        horizontal.target_position = {shaped.target_position.x, shaped.target_position.y};
        horizontal.target_velocity = {shaped.feedforward_velocity->x, shaped.feedforward_velocity->y};
        return run(input.state, altitude, horizontal, input.target_yaw_rad, input.target_yaw_rate_rad_sec, dt_sec);
    }
    AltitudeControlInput altitude{};
    altitude.mode = AltitudeControlMode::Position;
    altitude.target_altitude = -input.target_position.z;
    if (input.feedforward_velocity) {
        altitude.target_velocity.vz = -input.feedforward_velocity->z;
    }
    HorizontalPositionControlInput horizontal{};
    horizontal.mode = HorizontalControlMode::Position;
    horizontal.target_position = {input.target_position.x, input.target_position.y};
    if (input.feedforward_velocity) {
        horizontal.target_velocity = {input.feedforward_velocity->x, input.feedforward_velocity->y};
    }
    return run(input.state, altitude, horizontal, input.target_yaw_rad, input.target_yaw_rate_rad_sec, dt_sec);
}

PositionControl3DOutput ApPositionControl3DBackend::run_velocity(
    const PositionControl3DVelocityInput& input, double dt_sec)
{
    // Velocity targets are not shaped here; a later position target restarts the path from the vehicle.
    shaping_started_ = false;
    altitude_.set_acceleration_feedforward(0.0);
    horizontal_.set_acceleration_feedforward(0.0, 0.0);
    AltitudeControlInput altitude{};
    altitude.mode = AltitudeControlMode::Velocity;
    altitude.target_velocity.vz = -input.target_velocity.z;
    HorizontalPositionControlInput horizontal{};
    horizontal.mode = HorizontalControlMode::Velocity;
    horizontal.target_velocity = {input.target_velocity.x, input.target_velocity.y};
    return run(input.state, altitude, horizontal, input.target_yaw_rad, input.target_yaw_rate_rad_sec, dt_sec);
}

}  // namespace hakoniwa::drone::control_adapter
