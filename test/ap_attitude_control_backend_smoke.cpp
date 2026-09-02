// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_attitude_control_backend.hpp"

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

bool near(double lhs, double rhs, double tolerance = 1.0e-9)
{
    return std::abs(lhs - rhs) <= tolerance;
}

adapter::AttitudeQuaternion axis_angle(double x, double y, double z, double angle)
{
    const double sine = std::sin(angle * 0.5);
    return {std::cos(angle * 0.5), x * sine, y * sine, z * sine};
}

adapter::AttitudeControlInput base_input()
{
    adapter::AttitudeControlInput input{};
    input.dt_sec = 0.003;
    return input;
}

void proportional_roll_and_quaternion_sign()
{
    adapter::ApAttitudeControlBackendConfig config{};
    config.use_sqrt_controller = false;
    adapter::ApAttitudeControlBackend backend(config);

    auto input = base_input();
    input.target_attitude = axis_angle(1.0, 0.0, 0.0, 0.1);
    const auto positive = backend.run(input);
    require(near(positive.p, 0.45), "Roll error must use ArduPilot angle P gain");
    require(near(positive.q, 0.0), "Pure roll target must not produce pitch rate");

    input.target_attitude.w *= -1.0;
    input.target_attitude.x *= -1.0;
    input.target_attitude.y *= -1.0;
    input.target_attitude.z *= -1.0;
    const auto negative = backend.run(input);
    require(near(negative.p, positive.p), "Quaternion q and -q must be equivalent");
    require(near(negative.q, positive.q), "Quaternion sign must preserve pitch output");
    require(near(negative.r, positive.r), "Quaternion sign must preserve yaw output");
}

void yaw_wrap_and_feedforward()
{
    adapter::ApAttitudeControlBackendConfig config{};
    config.use_sqrt_controller = false;
    config.rate_yaw_p = 0.0;
    adapter::ApAttitudeControlBackend backend(config);

    auto input = base_input();
    input.attitude = axis_angle(0.0, 0.0, 1.0, 179.0 * pi / 180.0);
    input.target_attitude = axis_angle(0.0, 0.0, 1.0, -179.0 * pi / 180.0);
    const auto wrapped = backend.run(input);
    require(near(wrapped.r, 4.5 * 2.0 * pi / 180.0, 1.0e-8),
        "Yaw error must take the short path across pi");

    input.attitude = {};
    input.target_attitude = {};
    input.target_yaw_rate_rad_sec = 0.3;
    const auto feedforward = backend.run(input);
    require(near(feedforward.r, 0.3), "Yaw-rate feed-forward must reach body rate target");
}

void large_tilt_suppresses_yaw()
{
    adapter::ApAttitudeControlBackendConfig config{};
    config.use_sqrt_controller = false;
    adapter::ApAttitudeControlBackend backend(config);

    auto input = base_input();
    input.target_attitude = axis_angle(1.0, 0.0, 0.0, 70.0 * pi / 180.0);
    input.target_yaw_rate_rad_sec = 0.8;
    input.rate.r = -0.2;
    const auto output = backend.run(input);
    require(near(output.r, -0.2),
        "Above 60 degrees thrust error yaw must follow measured gyro rate");
    require(near(backend.get_status().feedforward_scalar, 0.0),
        "Large tilt must suppress feed-forward");
}

void rate_limit_and_reset()
{
    adapter::ApAttitudeControlBackendConfig config{};
    config.use_sqrt_controller = false;
    config.rate_roll_max_rad_sec = 0.2;
    config.rate_pitch_max_rad_sec = 0.4;
    adapter::ApAttitudeControlBackend backend(config);

    auto input = base_input();
    input.target_attitude = axis_angle(1.0, 0.0, 0.0, 0.5);
    const auto output = backend.run(input);
    require(near(output.p, 0.2), "Roll body-rate target must obey configured limit");
    require(backend.get_status().thrust_error_angle_rad > 0.0,
        "Run must publish thrust-error diagnostics");

    backend.reset();
    require(near(backend.get_status().thrust_error_angle_rad, 0.0),
        "Reset must clear attitude diagnostics");
    require(near(backend.get_status().feedforward_scalar, 1.0),
        "Reset must restore feed-forward diagnostic state");
}

void sqrt_controller_limits_large_error()
{
    adapter::ApAttitudeControlBackendConfig config{};
    config.accel_roll_max_rad_sec2 = 1.0;
    adapter::ApAttitudeControlBackend backend(config);

    auto input = base_input();
    input.target_attitude = axis_angle(1.0, 0.0, 0.0, 0.5);
    const auto output = backend.run(input);
    const double limited_acceleration = 40.0 * pi / 180.0;
    const double linear_distance = limited_acceleration / (4.5 * 4.5);
    const double expected = std::sqrt(
        2.0 * limited_acceleration * (0.5 - linear_distance * 0.5));
    require(near(output.p, expected, 1.0e-8),
        "Large attitude error must use ArduPilot sqrt-controller shaping");
}

void invalid_input_is_rejected()
{
    adapter::ApAttitudeControlBackend backend;
    auto input = base_input();
    input.dt_sec = 0.0;
    bool rejected_dt = false;
    try {
        (void)backend.run(input);
    } catch (const std::invalid_argument&) {
        rejected_dt = true;
    }
    require(rejected_dt, "Non-positive attitude dt must be rejected");

    input.dt_sec = 0.003;
    input.attitude = {0.0, 0.0, 0.0, 0.0};
    bool rejected_quaternion = false;
    try {
        (void)backend.run(input);
    } catch (const std::invalid_argument&) {
        rejected_quaternion = true;
    }
    require(rejected_quaternion, "Zero quaternion must be rejected");
}

}  // namespace

int main()
{
    proportional_roll_and_quaternion_sign();
    yaw_wrap_and_feedforward();
    large_tilt_suppresses_yaw();
    rate_limit_and_reset();
    sqrt_controller_limits_large_error();
    invalid_input_is_rejected();
    std::cout << "ap_attitude_control_backend_smoke: PASS\n";
    return 0;
}
