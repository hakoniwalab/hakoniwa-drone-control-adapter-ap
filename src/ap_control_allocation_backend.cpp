// SPDX-License-Identifier: GPL-3.0-or-later

// The desaturation sequence in this file is adapted from
// AP_MotorsMatrix::output_armed_stabilizing() and
// AP_MotorsMatrix::normalise_rpy_factors() in ArduPilot Copter 4.6.3,
// commit 92b0cd788ec29406f26c6f9c31d5ceedbd1cc538.
//
// The vehicle/HAL parts of AP_MotorsMatrix (PWM output, battery compensation,
// spool state and failed-motor detection) are intentionally outside this pure
// allocation backend. Its output is ArduPilot normalized motor thrust before
// MOT_THST_EXPO/spin-range conversion to an ESC actuator command.

#include "hakoniwa/drone/control_adapter/ap_control_allocation_backend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {

namespace {

constexpr std::size_t kArduPilotMaxMotors = 12;
constexpr double kEpsilon = 1e-9;
constexpr double kGeometryTolerance = 1e-6;

struct AllocationModel {
    std::array<double, kArduPilotMaxMotors> roll{};
    std::array<double, kArduPilotMaxMotors> pitch{};
    std::array<double, kArduPilotMaxMotors> yaw{};
    std::array<double, kArduPilotMaxMotors> throttle{};
    std::size_t count{0};
    bool roll_available{false};
    bool pitch_available{false};
    bool yaw_available{false};
};

struct AllocationLimits {
    bool roll{false};
    bool pitch{false};
    bool yaw{false};
    bool throttle_lower{false};
    bool throttle_upper{false};
};

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

void validate_config(const ApControlAllocationBackendConfig& config)
{
    if (!std::isfinite(config.throttle_rpy_mix)
        || config.throttle_rpy_mix < 0.0
        || config.throttle_rpy_mix > 1.0) {
        throw std::invalid_argument("ArduPilot allocator throttle_rpy_mix must be in [0,1]");
    }
    if (!std::isfinite(config.yaw_headroom)
        || config.yaw_headroom < 0.0
        || config.yaw_headroom > 1.0) {
        throw std::invalid_argument("ArduPilot allocator yaw_headroom must be in [0,1]");
    }
}

void validate_actuator_contract(const RotorActuatorConfig& actuator)
{
    const auto& position = actuator.geometry.position;
    const auto& axis = actuator.geometry.axis;
    if (!std::isfinite(position.x) || !std::isfinite(position.y)
        || !std::isfinite(position.z) || !std::isfinite(axis.x)
        || !std::isfinite(axis.y) || !std::isfinite(axis.z)
        || !std::isfinite(actuator.geometry.thrust_coefficient)
        || !std::isfinite(actuator.geometry.moment_ratio)
        || !std::isfinite(actuator.limit.min)
        || !std::isfinite(actuator.limit.max)
        || !std::isfinite(actuator.trim)
        || !std::isfinite(actuator.linearization_point)) {
        throw std::invalid_argument("ArduPilot allocator requires finite actuator geometry");
    }
    if (actuator.limit.min > actuator.limit.max) {
        throw std::invalid_argument("ArduPilot allocator actuator minimum exceeds maximum");
    }
}

bool normalize_axis(
    std::array<double, kArduPilotMaxMotors>& factors,
    std::size_t count,
    double normalized_max)
{
    double maximum = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        maximum = std::max(maximum, std::abs(factors[i]));
    }
    if (maximum <= kEpsilon) {
        return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
        factors[i] = normalized_max * factors[i] / maximum;
    }
    return true;
}

AllocationModel build_model(const ControlAllocationInput& input)
{
    if (input.actuator_count == 0) {
        return {};
    }
    if (input.actuator_count > kArduPilotMaxMotors) {
        throw std::invalid_argument(
            "ArduPilot Copter 4.6.3 supports at most 12 matrix motors");
    }

    AllocationModel model{};
    model.count = input.actuator_count;

    for (std::size_t i = 0; i < model.count; ++i) {
        const auto& actuator = input.actuators[i];
        validate_actuator_contract(actuator);

        const auto& position = actuator.geometry.position;
        const auto& axis = actuator.geometry.axis;
        const double norm = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
        if (!std::isfinite(norm) || norm <= kEpsilon) {
            throw std::invalid_argument("ArduPilot allocator requires a non-zero rotor axis");
        }

        const double ax = axis.x / norm;
        const double ay = axis.y / norm;
        const double az = axis.z / norm;
        if (std::abs(ax) > kGeometryTolerance
            || std::abs(ay) > kGeometryTolerance
            || az >= -1.0 + kGeometryTolerance) {
            throw std::invalid_argument(
                "ArduPilot matrix allocator currently supports downward FRD rotor axes only");
        }

        // r x axis - moment_ratio * axis, matching the public geometry and
        // ArduPilot CW/CCW yaw-factor convention.
        model.roll[i] = position.y * az - position.z * ay
            - actuator.geometry.moment_ratio * ax;
        model.pitch[i] = position.z * ax - position.x * az
            - actuator.geometry.moment_ratio * ay;
        model.yaw[i] = position.x * ay - position.y * ax
            - actuator.geometry.moment_ratio * az;
        model.throttle[i] = -az;
    }

    // This is the normalization performed by AP_MotorsMatrix for scripting
    // matrices: R/P/Y factors use [-0.5,+0.5], throttle uses [0,1].
    model.roll_available = normalize_axis(model.roll, model.count, 0.5);
    model.pitch_available = normalize_axis(model.pitch, model.count, 0.5);
    model.yaw_available = normalize_axis(model.yaw, model.count, 0.5);
    (void)normalize_axis(model.throttle, model.count, 1.0);
    return model;
}

ControlAllocationOutput make_empty_output(const ControlAllocationInput& input)
{
    ControlAllocationOutput output{};
    output.status.unallocated_torque_x = input.command.torque_x;
    output.status.unallocated_torque_y = input.command.torque_y;
    output.status.unallocated_torque_z = input.command.torque_z;
    output.status.unallocated_thrust_body_z = input.command.thrust.body_z;
    output.status.clipped = std::abs(input.command.torque_x) > kEpsilon
        || std::abs(input.command.torque_y) > kEpsilon
        || std::abs(input.command.torque_z) > kEpsilon
        || std::abs(input.command.thrust.body_z) > kEpsilon;
    return output;
}

}  // namespace

ApControlAllocationBackend::ApControlAllocationBackend(
    const ApControlAllocationBackendConfig& config)
    : config_(config)
{
    validate_config(config_);
}

void ApControlAllocationBackend::reset()
{
    // AP_MotorsMatrix allocation has no cross-step state when failed-motor and
    // spool-state handling remain at the vehicle layer.
}

ControlAllocationOutput ApControlAllocationBackend::run(
    const ControlAllocationInput& input)
{
    validate_config(config_);
    if (!std::isfinite(input.command.thrust.body_z)
        || !std::isfinite(input.command.torque_x)
        || !std::isfinite(input.command.torque_y)
        || !std::isfinite(input.command.torque_z)) {
        throw std::invalid_argument("ArduPilot allocator requires a finite control command");
    }
    const AllocationModel model = build_model(input);
    if (model.count == 0) {
        return make_empty_output(input);
    }

    const double requested_roll = input.command.torque_x;
    const double requested_pitch = input.command.torque_y;
    const double requested_yaw = input.command.torque_z;
    const double roll_thrust = model.roll_available ? requested_roll : 0.0;
    const double pitch_thrust = model.pitch_available ? requested_pitch : 0.0;
    double yaw_thrust = model.yaw_available ? requested_yaw : 0.0;
    // Public thrust is normalized by hover thrust.  The public upper limit is
    // T_max/T_hover, so its reciprocal is AP's hover fraction.  Allocation
    // deliberately does not use the configuration-file MOT_THST_HOVER.
    const double actuator_limit_max = input.actuators[0].limit.max;
    if (!std::isfinite(actuator_limit_max) || actuator_limit_max <= 0.0) {
        throw std::invalid_argument("ArduPilot allocator requires a positive actuator maximum");
    }
    for (std::size_t i = 1; i < model.count; ++i) {
        if (std::abs(input.actuators[i].limit.max - actuator_limit_max) > kGeometryTolerance) {
            throw std::invalid_argument("ArduPilot matrix allocator requires equal actuator maximums");
        }
    }
    const double hover_thrust = 1.0 / actuator_limit_max;
    const double requested_throttle = -input.command.thrust.body_z * hover_thrust;
    double throttle_thrust = requested_throttle;
    AllocationLimits limits{};
    limits.roll = !model.roll_available && std::abs(requested_roll) > kEpsilon;
    limits.pitch = !model.pitch_available && std::abs(requested_pitch) > kEpsilon;
    limits.yaw = !model.yaw_available && std::abs(requested_yaw) > kEpsilon;

    if (throttle_thrust <= 0.0) {
        throttle_thrust = 0.0;
        limits.throttle_lower = true;
    }
    if (throttle_thrust >= 1.0) {
        throttle_thrust = 1.0;
        limits.throttle_upper = true;
    }

    double throttle_avg_max = std::max(
        throttle_thrust,
        throttle_thrust * (1.0 - config_.throttle_rpy_mix)
            + hover_thrust * config_.throttle_rpy_mix);
    throttle_avg_max = clamp(throttle_avg_max, throttle_thrust, 1.0);
    double throttle_best_rpy = std::min(0.5, throttle_avg_max);

    std::array<double, kArduPilotMaxMotors> output{};
    double yaw_allowed = 1.0;
    for (std::size_t i = 0; i < model.count; ++i) {
        output[i] = roll_thrust * model.roll[i]
            + pitch_thrust * model.pitch[i];
        if (std::abs(model.yaw[i]) > kEpsilon) {
            const double rp_best = throttle_best_rpy + output[i];
            const double room = (yaw_thrust * model.yaw[i] > 0.0)
                ? 1.0 - rp_best
                : rp_best;
            yaw_allowed = std::min(
                yaw_allowed,
                std::max(room, 0.0) / std::abs(model.yaw[i]));
        }
    }

    yaw_allowed = std::max(yaw_allowed, config_.yaw_headroom);
    if (std::abs(yaw_thrust) > yaw_allowed) {
        yaw_thrust = clamp(yaw_thrust, -yaw_allowed, yaw_allowed);
        limits.yaw = true;
    }

    double rpy_low = 1.0;
    double rpy_high = -1.0;
    for (std::size_t i = 0; i < model.count; ++i) {
        output[i] += yaw_thrust * model.yaw[i];
        rpy_low = std::min(rpy_low, output[i]);
        rpy_high = std::max(rpy_high, output[i]);
    }

    double rpy_scale = 1.0;
    if (rpy_high - rpy_low > 1.0) {
        rpy_scale = 1.0 / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0.0) {
        rpy_scale = std::min(rpy_scale, -throttle_avg_max / rpy_low);
    }

    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_best_rpy = -rpy_low;
    double throttle_adjustment = throttle_thrust - throttle_best_rpy;
    if (rpy_scale < 1.0) {
        limits.roll = true;
        limits.pitch = true;
        limits.yaw = true;
        if (throttle_adjustment > 0.0) {
            limits.throttle_upper = true;
        }
        throttle_adjustment = 0.0;
    } else if (throttle_adjustment < 0.0) {
        throttle_adjustment = 0.0;
    } else if (throttle_adjustment > 1.0 - (throttle_best_rpy + rpy_high)) {
        throttle_adjustment = 1.0 - (throttle_best_rpy + rpy_high);
        limits.throttle_upper = true;
    }

    const double allocated_throttle = throttle_best_rpy + throttle_adjustment;
    ControlAllocationOutput result{};
    result.actuator_commands.count = model.count;
    double output_sum = 0.0;
    for (std::size_t i = 0; i < model.count; ++i) {
        const double ap_fraction = allocated_throttle * model.throttle[i]
            + rpy_scale * output[i];
        double hover_units = ap_fraction / hover_thrust;
        const auto& actuator = input.actuators[i];
        // Trim and the linearization point are expressed in the same public
        // hover unit as the output. AP's linear mixer is affine around that
        // point; their difference is the constant bias at this boundary.
        hover_units += actuator.trim - actuator.linearization_point;
        const double limited = clamp(
            hover_units, actuator.limit.min, actuator.limit.max);
        if (std::abs(limited - hover_units) > kEpsilon) {
            result.status.clipped = true;
        }
        result.actuator_commands.values[i] = limited;
        output_sum += limited * hover_thrust;
    }

    result.status.clipped = result.status.clipped || limits.roll || limits.pitch || limits.yaw
        || limits.throttle_lower || limits.throttle_upper;
    result.status.unallocated_torque_x = requested_roll - roll_thrust * rpy_scale;
    result.status.unallocated_torque_y = requested_pitch - pitch_thrust * rpy_scale;
    result.status.unallocated_torque_z = requested_yaw - yaw_thrust * rpy_scale;
    const double allocated_collective = output_sum / static_cast<double>(model.count);
    result.status.unallocated_thrust_body_z =
        input.command.thrust.body_z + allocated_collective / hover_thrust;
    return result;
}

void ApControlAllocationBackend::set_config(
    const ApControlAllocationBackendConfig& config)
{
    validate_config(config);
    config_ = config;
}

}  // namespace hakoniwa::drone::control_adapter
