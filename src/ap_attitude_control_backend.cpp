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

// AC_AttitudeControl::thrust_vector_rotation_angles, without the yaw-error limit:
// the rotation from `from` to `to` as thrust-vector (x, y) and heading (z) angles in `from`'s frame.
Vec3 thrust_heading_error(const Quat& from, const Quat& to)
{
    const Vec3 thrust_up{0.0, 0.0, -1.0};
    const Vec3 to_thrust = rotate(to, thrust_up);
    const Vec3 from_thrust = rotate(from, thrust_up);
    Vec3 axis = cross(from_thrust, to_thrust);
    const double angle = std::acos(std::clamp(dot(from_thrust, to_thrust), -1.0, 1.0));
    const double length = norm(axis);
    axis = (length <= epsilon || angle <= epsilon) ? thrust_up : axis * (1.0 / length);
    axis = rotate(inverse(from), axis);
    const Quat thrust_correction = from_axis_angle(axis, angle);
    const Vec3 rotation = to_axis_angle(thrust_correction);
    const Quat heading_correction = inverse(thrust_correction) * inverse(from) * to;
    return {rotation.x, rotation.y, to_axis_angle(heading_correction).z};
}

// AC_AttitudeControl::input_shaping_ang_vel
double input_shaping_ang_vel(double target_ang_vel, double desired_ang_vel, double accel_max, double dt, double input_tc)
{
    if (input_tc > 0.0) {
        const double error_rate = desired_ang_vel - target_ang_vel;
        const double desired_ang_accel = sqrt_controller(error_rate, 1.0 / std::max(input_tc, 0.01), 0.0, dt);
        desired_ang_vel = target_ang_vel + desired_ang_accel * dt;
    }
    if (accel_max > 0.0) {
        const double delta = accel_max * dt;
        return std::clamp(desired_ang_vel, target_ang_vel - delta, target_ang_vel + delta);
    }
    return desired_ang_vel;
}

// AC_AttitudeControl::input_shaping_angle
double input_shaping_angle(double error_angle, double input_tc, double accel_max, double target_ang_vel,
                           double desired_ang_vel, double max_ang_vel, double dt)
{
    desired_ang_vel += sqrt_controller(error_angle, 1.0 / std::max(input_tc, 0.01), accel_max, dt);
    if (max_ang_vel > 0.0) {
        desired_ang_vel = std::clamp(desired_ang_vel, -max_ang_vel, max_ang_vel);
    }
    return input_shaping_ang_vel(target_ang_vel, desired_ang_vel, accel_max, dt, 0.0);
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
    shaping_started_ = false;
    ang_vel_target_x_ = ang_vel_target_y_ = ang_vel_target_z_ = 0.0;
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
    Quat target = normalize(input.target_attitude);
    // Earth-frame angular velocity fed forward: the shaped target's, or the commanded yaw rate.
    Vec3 feedforward_earth{0.0, 0.0, input.target_yaw_rate_rad_sec};
    if (config_.rate_feedforward_enabled) {
        // AC_AttitudeControl::input_thrust_vector_heading with _rate_bf_ff_enabled:
        // update_attitude_target(), then shape the target's angular velocity toward the command.
        if (!shaping_started_) {
            target_w_ = body.w; target_x_ = body.x; target_y_ = body.y; target_z_ = body.z;
            shaping_started_ = true;
        }
        const double dt = input.dt_sec;
        Vec3 ang_vel{ang_vel_target_x_, ang_vel_target_y_, ang_vel_target_z_};
        Quat shaped{target_w_, target_x_, target_y_, target_z_};
        const double ang_vel_length = norm(ang_vel);
        if (ang_vel_length > epsilon) {
            shaped = shaped * from_axis_angle(ang_vel * (1.0 / ang_vel_length), ang_vel_length * dt);
        }
        shaped = normalize({shaped.w, shaped.x, shaped.y, shaped.z});
        const Vec3 error = thrust_heading_error(shaped, target);
        const double slew_yaw = config_.rate_yaw_max_rad_sec > epsilon
            ? std::min(config_.rate_yaw_max_rad_sec, config_.slew_yaw_rad_sec) : config_.slew_yaw_rad_sec;
        const double heading_rate = std::clamp(input.target_yaw_rate_rad_sec, -slew_yaw, slew_yaw);
        ang_vel.x = input_shaping_angle(error.x, config_.input_time_constant_sec, config_.accel_roll_max_rad_sec2,
                                        ang_vel.x, 0.0, 0.0, dt);
        ang_vel.y = input_shaping_angle(error.y, config_.input_time_constant_sec, config_.accel_pitch_max_rad_sec2,
                                        ang_vel.y, 0.0, 0.0, dt);
        ang_vel.z = input_shaping_angle(error.z, config_.input_time_constant_sec, config_.accel_yaw_max_rad_sec2,
                                        ang_vel.z, heading_rate, slew_yaw, dt);
        // AC_AttitudeControl::ang_vel_limit
        ApAttitudeControlBackendConfig limits = config_;
        limits.rate_yaw_max_rad_sec = slew_yaw;
        limit_angular_rate(ang_vel, limits);
        ang_vel_target_x_ = ang_vel.x; ang_vel_target_y_ = ang_vel.y; ang_vel_target_z_ = ang_vel.z;
        target_w_ = shaped.w; target_x_ = shaped.x; target_y_ = shaped.y; target_z_ = shaped.z;
        target = shaped;
        feedforward_earth = rotate(shaped, ang_vel);
    }
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

    Vec3 desired_body_rate = feedforward_earth;
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
