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

void altitude_throttle_filter_has_step_response_and_reset()
{
    adapter::ApAltitudeControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.acceleration_p = 1.0;
    config.acceleration_i = config.acceleration_d = config.acceleration_feed_forward = 0.0;
    config.acceleration_target_filter_hz = config.acceleration_error_filter_hz = 0.0;
    config.acceleration_derivative_filter_hz = 0.0;
    config.hover_thrust = 0.5;
    config.thrust_max = 2.0;
    config.throttle_filter_hz = 2.0;
    adapter::ApAltitudeControlBackend backend(config);

    constexpr double dt = 0.01;
    adapter::AltitudeControlInput input{};
    input.mode = adapter::AltitudeControlMode::Velocity;
    require(std::abs(backend.run(input, dt).body_z + 1.0) < 1.0e-12,
        "Throttle filter must start from the first in-air throttle value");

    input.target_velocity.vz = 1.0;
    const double alpha = dt / (dt + 1.0 / (2.0 * pi * config.throttle_filter_hz));
    const auto first_step = backend.run(input, dt);
    require(std::abs(first_step.body_z - (-1.0 - 0.2 * alpha)) < 1.0e-9,
        "First throttle step must move by the 2 Hz low-pass alpha");
    auto settled = first_step;
    for (int i = 0; i < 1000; ++i) {
        settled = backend.run(input, dt);
    }
    require(std::abs(settled.body_z + 1.2) < 1.0e-6,
        "Filtered throttle must converge to the commanded throttle");

    backend.reset();
    require(std::abs(backend.run(input, dt).body_z + 1.2) < 1.0e-12,
        "Reset throttle filter must initialize from the next throttle command");

    config.throttle_filter_hz = 0.0;
    backend.set_config(config);
    input.target_velocity.vz = 0.0;
    (void)backend.run(input, dt);
    input.target_velocity.vz = 1.0;
    require(std::abs(backend.run(input, dt).body_z + 1.2) < 1.0e-12,
        "Zero throttle filter frequency must disable filtering");
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

void position_shaping_respects_speed_acceleration_and_stops()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.position_p = 1.0;
    horizontal.velocity_p = 1.0;
    horizontal.velocity_i = horizontal.velocity_d = horizontal.velocity_feed_forward = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    horizontal.speed_max_mps = 5.0;
    horizontal.angle_max_rad = 1.0;
    adapter::ApPositionShapingConfig shaping{true, 2.0, 20.0, 2.5, 20.0};
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping);

    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 30.0;
    constexpr double dt = 0.01;
    double previous_velocity = 0.0;
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    // A unit-mass point vehicle follows the requested pitch acceleration. This
    // exercises the shaper, position loop and stopping trajectory together.
    for (int i = 0; i < 2000; ++i) {
        const auto output = backend.run_position(input, dt);
        const auto& q = output.target_attitude;
        const double pitch = std::asin(std::max(-1.0, std::min(1.0,
            2.0 * (q.w * q.y - q.z * q.x))));
        const double acceleration = -9.80665 * std::tan(pitch);
        input.state.velocity.x += acceleration * dt;
        input.state.position.x += input.state.velocity.x * dt;
        max_velocity = std::max(max_velocity, std::abs(input.state.velocity.x));
        max_acceleration = std::max(max_acceleration,
            std::abs((input.state.velocity.x - previous_velocity) / dt));
        previous_velocity = input.state.velocity.x;
    }
    require(max_velocity <= horizontal.speed_max_mps + 0.08,
        "Shaped 30 m move must not exceed horizontal speed_max_mps");
    require(max_acceleration <= shaping.acceleration_xy_mps2 + 0.08,
        "Shaped 30 m move must not exceed WPNAV_ACCEL in SI units");
    require(std::abs(input.state.position.x - 30.0) < 0.15,
        "Shaped 30 m move must arrive at the target");
    require(std::abs(input.state.velocity.x) < 0.05,
        "Shaped 30 m move must stop at the target");
}

void disabled_position_shaping_passes_target_directly()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.position_p = 1.0;
    horizontal.velocity_p = 1.0;
    horizontal.velocity_i = horizontal.velocity_d = horizontal.velocity_feed_forward = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    horizontal.speed_max_mps = 100.0;
    horizontal.angle_max_rad = 1.0;
    adapter::ApPositionControl3DBackend backend(
        altitude, horizontal, adapter::ApPositionShapingConfig{false});
    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 30.0;
    const auto output = backend.run_position(input, 0.01);
    const double pitch = std::atan2(
        2.0 * (output.target_attitude.w * output.target_attitude.y
             - output.target_attitude.z * output.target_attitude.x),
        1.0 - 2.0 * (output.target_attitude.x * output.target_attitude.x
                   + output.target_attitude.y * output.target_attitude.y));
    require(std::abs(pitch + horizontal.angle_max_rad) < 1.0e-9,
        "position_input_shaping 0 must hand the full target directly to the position P loop");
}


}  // namespace

int main()
{
    altitude_feedback_is_not_jerk_limited();
    altitude_throttle_filter_has_step_response_and_reset();
    horizontal_feedback_is_limited_only_by_lean_angle();
    position_3d_uses_target_heading_for_tilt();
    position_shaping_respects_speed_acceleration_and_stops();
    disabled_position_shaping_passes_target_directly();
    std::cout << "ap_position_control_backends_smoke: PASS\n";
    return 0;
}
