// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"

#include <cmath>

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
    const ApHorizontalPositionControlBackendConfig& horizontal)
    : altitude_(altitude), horizontal_(horizontal)
{
}

void ApPositionControl3DBackend::reset()
{
    altitude_.reset();
    horizontal_.reset();
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
    output.thrust = thrust;
    output.target_yaw_rate_rad_sec = target_yaw_rate_rad_sec;
    return output;
}

PositionControl3DOutput ApPositionControl3DBackend::run_position(
    const PositionControl3DPositionInput& input, double dt_sec)
{
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
    AltitudeControlInput altitude{};
    altitude.mode = AltitudeControlMode::Velocity;
    altitude.target_velocity.vz = -input.target_velocity.z;
    HorizontalPositionControlInput horizontal{};
    horizontal.mode = HorizontalControlMode::Velocity;
    horizontal.target_velocity = {input.target_velocity.x, input.target_velocity.y};
    return run(input.state, altitude, horizontal, input.target_yaw_rad, input.target_yaw_rate_rad_sec, dt_sec);
}

}  // namespace hakoniwa::drone::control_adapter
