// SPDX-License-Identifier: GPL-3.0-or-later

// This is a dependency-free extraction of the quaternion attitude correction
// in ArduPilot Copter 4.6.3 AC_AttitudeControl. It preserves the ordered
// thrust-vector/heading correction, yaw-error limiting, sqrt angle controller,
// angular-rate limiting, and large-tilt feed-forward suppression.

#include "hakoniwa/drone/control_adapter/ap_attitude_control_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {
namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double thrust_error_limit = pi / 6.0;
constexpr double yaw_error_limit = pi / 4.0;
constexpr double accel_rp_min = 40.0 * pi / 180.0;
constexpr double accel_rp_max = 720.0 * pi / 180.0;
constexpr double accel_yaw_min = 10.0 * pi / 180.0;
constexpr double accel_yaw_max = 120.0 * pi / 180.0;
constexpr double epsilon = 1.0e-12;

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

struct Quat {
    double w{1.0};
    double x{};
    double y{};
    double z{};
};

Vec3 operator+(const Vec3& lhs, const Vec3& rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator*(const Vec3& value, double scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

double dot(const Vec3& lhs, const Vec3& rhs)
{
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

double norm(const Vec3& value)
{
    return std::sqrt(dot(value, value));
}

Quat operator*(const Quat& lhs, const Quat& rhs)
{
    return {
        lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
        lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w
    };
}

Quat inverse(const Quat& value)
{
    return {value.w, -value.x, -value.y, -value.z};
}

Quat normalize(const AttitudeQuaternion& value)
{
    if (!std::isfinite(value.w) || !std::isfinite(value.x)
        || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        throw std::invalid_argument("ArduPilot attitude quaternion must be finite");
    }
    const double length = std::sqrt(
        value.w * value.w + value.x * value.x
        + value.y * value.y + value.z * value.z);
    if (length <= epsilon) {
        throw std::invalid_argument("ArduPilot attitude quaternion must be non-zero");
    }
    return {value.w / length, value.x / length, value.y / length, value.z / length};
}

Vec3 rotate(const Quat& rotation, const Vec3& value)
{
    const Quat rotated = rotation * Quat{0.0, value.x, value.y, value.z}
        * inverse(rotation);
    return {rotated.x, rotated.y, rotated.z};
}

Quat from_axis_angle(const Vec3& axis, double angle)
{
    if (std::abs(angle) <= epsilon) {
        return {};
    }
    const double half = angle * 0.5;
    const double sine = std::sin(half);
    return {std::cos(half), axis.x * sine, axis.y * sine, axis.z * sine};
}

Vec3 to_axis_angle(const Quat& value)
{
    const double vector_length = std::sqrt(
        value.x * value.x + value.y * value.y + value.z * value.z);
    if (vector_length <= epsilon) {
        return {};
    }
    double angle = 2.0 * std::atan2(vector_length, value.w);
    angle = std::remainder(angle, 2.0 * pi);
    return {
        value.x * angle / vector_length,
        value.y * angle / vector_length,
        value.z * angle / vector_length
    };
}

double sqrt_controller(double error, double p, double second_order_limit, double dt)
{
    double correction = 0.0;
    if (second_order_limit <= 0.0) {
        correction = error * p;
    } else if (std::abs(p) <= epsilon) {
        correction = std::copysign(
            std::sqrt(std::max(0.0, 2.0 * second_order_limit * std::abs(error))),
            error);
    } else {
        const double linear_distance = second_order_limit / (p * p);
        if (error > linear_distance) {
            correction = std::sqrt(
                2.0 * second_order_limit * (error - linear_distance * 0.5));
        } else if (error < -linear_distance) {
            correction = -std::sqrt(
                2.0 * second_order_limit * (-error - linear_distance * 0.5));
        } else {
            correction = error * p;
        }
    }
    return std::clamp(correction, -std::abs(error) / dt, std::abs(error) / dt);
}

double inv_sqrt_controller(double output, double p, double acceleration_limit)
{
    if (acceleration_limit > 0.0 && std::abs(p) <= epsilon) {
        return output * output / (2.0 * acceleration_limit);
    }
    if (acceleration_limit <= 0.0 && std::abs(p) > epsilon) {
        return output / p;
    }
    if (acceleration_limit <= 0.0) {
        return 0.0;
    }
    const double linear_velocity = acceleration_limit / p;
    if (std::abs(output) < linear_velocity) {
        return output / p;
    }
    const double linear_distance = acceleration_limit / (p * p);
    const double stopping_distance = linear_distance * 0.5
        + output * output / (2.0 * acceleration_limit);
    return std::copysign(stopping_distance, output);
}

void limit_angular_rate(Vec3& rate, const ApAttitudeControlBackendConfig& config)
{
    const double roll_max = config.rate_roll_max_rad_sec;
    const double pitch_max = config.rate_pitch_max_rad_sec;
    if (roll_max <= epsilon || pitch_max <= epsilon) {
        if (roll_max > epsilon) {
            rate.x = std::clamp(rate.x, -roll_max, roll_max);
        }
        if (pitch_max > epsilon) {
            rate.y = std::clamp(rate.y, -pitch_max, pitch_max);
        }
    } else {
        const double scaled_length = std::hypot(rate.x / roll_max, rate.y / pitch_max);
        if (scaled_length > 1.0) {
            rate.x /= scaled_length;
            rate.y /= scaled_length;
        }
    }
    if (config.rate_yaw_max_rad_sec > epsilon) {
        rate.z = std::clamp(
            rate.z, -config.rate_yaw_max_rad_sec, config.rate_yaw_max_rad_sec);
    }
}

void validate_config(const ApAttitudeControlBackendConfig& config)
{
    const double values[] = {
        config.angle_roll_p, config.angle_pitch_p, config.angle_yaw_p,
        config.rate_yaw_p, config.accel_roll_max_rad_sec2,
        config.accel_pitch_max_rad_sec2, config.accel_yaw_max_rad_sec2,
        config.rate_roll_max_rad_sec, config.rate_pitch_max_rad_sec,
        config.rate_yaw_max_rad_sec
    };
    for (double value : values) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument(
                "ArduPilot attitude configuration values must be finite and non-negative");
        }
    }
}

double correction_rate(
    double error, double p, double acceleration, double min_acceleration,
    double max_acceleration, double dt, bool use_sqrt)
{
    if (!use_sqrt || acceleration <= epsilon) {
        return error * p;
    }
    return sqrt_controller(
        error, p, std::clamp(acceleration * 0.5, min_acceleration, max_acceleration), dt);
}

}  // namespace

ApAttitudeControlBackend::ApAttitudeControlBackend(
    const ApAttitudeControlBackendConfig& config)
{
    set_config(config);
}

void ApAttitudeControlBackend::reset()
{
    status_ = {};
}

AngularRateTarget ApAttitudeControlBackend::run(const AttitudeControlInput& input)
{
    if (!std::isfinite(input.dt_sec) || input.dt_sec <= 0.0) {
        throw std::invalid_argument("ArduPilot attitude control requires dt_sec > 0");
    }
    if (!std::isfinite(input.target_yaw_rate_rad_sec)
        || !std::isfinite(input.rate.p) || !std::isfinite(input.rate.q)
        || !std::isfinite(input.rate.r)) {
        throw std::invalid_argument("ArduPilot attitude control input must be finite");
    }

    const Quat body = normalize(input.attitude);
    const Quat desired = normalize(input.target_attitude);
    Quat target = desired;
    const Vec3 thrust_up{0.0, 0.0, -1.0};
    const Vec3 target_thrust = rotate(target, thrust_up);
    const Vec3 body_thrust = rotate(body, thrust_up);
    Vec3 thrust_axis = cross(body_thrust, target_thrust);
    const double thrust_error = std::acos(std::clamp(dot(body_thrust, target_thrust), -1.0, 1.0));
    const double axis_length = norm(thrust_axis);
    if (axis_length <= epsilon || thrust_error <= epsilon) {
        thrust_axis = thrust_up;
    } else {
        thrust_axis = thrust_axis * (1.0 / axis_length);
    }
    thrust_axis = rotate(inverse(body), thrust_axis);
    const Quat thrust_correction = from_axis_angle(thrust_axis, thrust_error);
    const Vec3 thrust_rotation = to_axis_angle(thrust_correction);
    Vec3 attitude_error{thrust_rotation.x, thrust_rotation.y, 0.0};

    const Quat heading_correction = inverse(thrust_correction) * inverse(body) * target;
    attitude_error.z = to_axis_angle(heading_correction).z;

    const double heading_accel = std::clamp(
        config_.accel_yaw_max_rad_sec2 * 0.5, accel_yaw_min, accel_yaw_max);
    if (config_.rate_yaw_p > epsilon) {
        const double max_heading_error = std::min(
            inv_sqrt_controller(
                1.0 / config_.rate_yaw_p,
                config_.angle_yaw_p,
                heading_accel),
            yaw_error_limit);
        if (config_.angle_yaw_p > epsilon
            && std::abs(attitude_error.z) > max_heading_error) {
            attitude_error.z = std::clamp(
                std::remainder(attitude_error.z, 2.0 * pi),
                -max_heading_error,
                max_heading_error);
            target = body * thrust_correction
                * from_axis_angle({0.0, 0.0, 1.0}, attitude_error.z);
        }
    }

    Vec3 correction{
        correction_rate(
            attitude_error.x, config_.angle_roll_p,
            config_.accel_roll_max_rad_sec2, accel_rp_min, accel_rp_max,
            input.dt_sec, config_.use_sqrt_controller),
        correction_rate(
            attitude_error.y, config_.angle_pitch_p,
            config_.accel_pitch_max_rad_sec2, accel_rp_min, accel_rp_max,
            input.dt_sec, config_.use_sqrt_controller),
        correction_rate(
            attitude_error.z, config_.angle_yaw_p,
            config_.accel_yaw_max_rad_sec2, accel_yaw_min, accel_yaw_max,
            input.dt_sec, config_.use_sqrt_controller)
    };
    limit_angular_rate(correction, config_);

    Vec3 desired_body_rate{0.0, 0.0, input.target_yaw_rate_rad_sec};
    limit_angular_rate(desired_body_rate, config_);
    const Vec3 body_feedforward = rotate(inverse(body), desired_body_rate);

    status_.thrust_error_angle_rad = thrust_error;
    status_.feedforward_scalar = 1.0;
    if (thrust_error > thrust_error_limit * 2.0) {
        correction.z = input.rate.r;
        status_.feedforward_scalar = 0.0;
    } else if (thrust_error > thrust_error_limit) {
        status_.feedforward_scalar = 1.0
            - (thrust_error - thrust_error_limit) / thrust_error_limit;
        correction.x += body_feedforward.x * status_.feedforward_scalar;
        correction.y += body_feedforward.y * status_.feedforward_scalar;
        correction.z += body_feedforward.z;
        correction.z = input.rate.r * (1.0 - status_.feedforward_scalar)
            + correction.z * status_.feedforward_scalar;
    } else {
        correction = correction + body_feedforward;
    }

    return {correction.x, correction.y, correction.z};
}

void ApAttitudeControlBackend::set_config(
    const ApAttitudeControlBackendConfig& config)
{
    validate_config(config);
    config_ = config;
    reset();
}

ApAttitudeControlBackendStatus ApAttitudeControlBackend::get_status() const
{
    return status_;
}

}  // namespace hakoniwa::drone::control_adapter
