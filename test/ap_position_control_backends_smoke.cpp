// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace adapter = hakoniwa::drone::control_adapter;

namespace {

constexpr double pi = 3.14159265358979323846;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void altitude_feedback_is_not_jerk_limited()
{
    adapter::ApAltitudeControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.acceleration_p = 0.5;
    config.acceleration_i = config.acceleration_d = config.acceleration_feed_forward = 0.0;
    config.acceleration_target_filter_hz = config.acceleration_error_filter_hz = 0.0;
    config.acceleration_derivative_filter_hz = 0.0;
    config.acceleration_max_mps2 = config.jerk_max_mps3 = 0.01;
    config.hover_thrust = 0.5;
    config.thrust_max = 2.0;
    adapter::ApAltitudeControlBackend backend(config);

    adapter::AltitudeControlInput input{};
    input.mode = adapter::AltitudeControlMode::Velocity;
    input.target_velocity.vz = 2.0;
    const auto output = backend.run(input, 0.01);

    require(output.body_z < -1.1,
        "First-cycle altitude feedback must not be limited to jerk*dt");
}

void horizontal_feedback_is_limited_only_by_lean_angle()
{
    adapter::ApHorizontalPositionControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_error_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.speed_max_mps = 100.0;
    config.acceleration_max_mps2 = config.jerk_max_mps3 = 0.01;
    config.angle_max_rad = 0.2;
    adapter::ApHorizontalPositionControlBackend backend(config);

    adapter::HorizontalPositionControlInput input{};
    input.mode = adapter::HorizontalControlMode::Velocity;
    input.target_velocity.vx = 100.0;
    const auto output = backend.run(input, 0.01);

    require(std::abs(output.pitch_rad + config.angle_max_rad) < 1.0e-9,
        "Horizontal feedback must reach the lean-angle limit without jerk limiting");
    require(std::abs(output.roll_rad) < 1.0e-12,
        "Pure north acceleration must not request roll at zero yaw");
}

adapter::Vector3D rotate(const adapter::AttitudeQuaternion& q, const adapter::Vector3D& v)
{
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty),
            v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

void position_3d_uses_target_heading_for_tilt()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.velocity_i = horizontal.velocity_d = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal);

    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 1.0;
    input.target_yaw_rad = pi;
    const auto output = backend.run_position(input, 0.01);
    const auto thrust_direction = rotate(output.target_attitude, {0.0, 0.0, -1.0});
    require(thrust_direction.x > 0.0,
        "With a 180-degree yaw target, world-frame thrust must still point toward +x error");

    backend.reset();
    input.target_yaw_rad.reset();
    input.target_position = {};
    input.state.yaw_rad = 0.7;
    const auto held = backend.run_position(input, 0.01);
    const double yaw = std::atan2(2.0 * (held.target_attitude.w * held.target_attitude.z
                                      + held.target_attitude.x * held.target_attitude.y),
                                  1.0 - 2.0 * (held.target_attitude.y * held.target_attitude.y
                                             + held.target_attitude.z * held.target_attitude.z));
    require(std::abs(yaw - input.state.yaw_rad) < 1.0e-9,
        "Missing yaw target must hold the current yaw");
}

}  // namespace

int main()
{
    altitude_feedback_is_not_jerk_limited();
    horizontal_feedback_is_limited_only_by_lean_angle();
    position_3d_uses_target_heading_for_tilt();
    std::cout << "ap_position_control_backends_smoke: PASS\n";
    return 0;
}
