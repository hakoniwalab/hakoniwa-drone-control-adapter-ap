// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_control_allocation_backend.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace adapter = hakoniwa::drone::control_adapter;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
    return std::abs(actual - expected) <= tolerance;
}

void configure_rotor(
    adapter::RotorActuatorConfig& rotor,
    double x,
    double y,
    double moment_ratio)
{
    rotor.geometry.position = {x, y, 0.0};
    rotor.geometry.axis = {0.0, 0.0, -1.0};
    rotor.geometry.moment_ratio = moment_ratio;
    rotor.limit = {0.0, 4.0};
}

adapter::ControlAllocationInput make_quadx(double throttle = 1.0)
{
    adapter::ControlAllocationInput input{};
    input.actuator_count = 4;
    input.command.thrust.body_z = -throttle;
    configure_rotor(input.actuators[0], 0.2, 0.2, 0.02);
    configure_rotor(input.actuators[1], -0.2, -0.2, 0.02);
    configure_rotor(input.actuators[2], 0.2, -0.2, -0.02);
    configure_rotor(input.actuators[3], -0.2, 0.2, -0.02);
    return input;
}

adapter::ControlAllocationInput make_hexa(double throttle = 1.0)
{
    adapter::ControlAllocationInput input{};
    input.actuator_count = 6;
    input.command.thrust.body_z = -throttle;
    constexpr double radius = 0.5;
    constexpr double pi = 3.14159265358979323846;
    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        const double angle = 2.0 * pi * static_cast<double>(i) / 6.0;
        const double direction = (i % 2 == 0) ? 0.03 : -0.03;
        configure_rotor(
            input.actuators[i],
            radius * std::cos(angle),
            radius * std::sin(angle),
            direction);
    }
    return input;
}

void verifies_collective_and_roll_allocation()
{
    adapter::ApControlAllocationBackend backend;
    const auto collective = backend.run(make_quadx());
    require(collective.actuator_commands.count == 4, "Quad output count mismatch");
    for (std::size_t i = 0; i < 4; ++i) {
        require(near(collective.actuator_commands.values[i], 1.0),
            "Quad collective output mismatch");
    }
    require(!collective.status.clipped, "Nominal collective should not clip");

    auto input = make_quadx();
    input.command.torque_x = 0.2;
    const auto roll = backend.run(input);
    require(near(roll.actuator_commands.values[0], 0.15 / 0.25),
        "Unexpected Quad/X motor 0 roll output");
    require(near(roll.actuator_commands.values[1], 0.35 / 0.25),
        "Unexpected Quad/X motor 1 roll output");
    require(near(roll.actuator_commands.values[2], 0.35 / 0.25),
        "Unexpected Quad/X motor 2 roll output");
    require(near(roll.actuator_commands.values[3], 0.15 / 0.25),
        "Unexpected Quad/X motor 3 roll output");
}

void verifies_hexa_and_yaw_desaturation()
{
    adapter::ApControlAllocationBackend backend;
    const auto collective = backend.run(make_hexa());
    require(collective.actuator_commands.count == 6, "Hexa output count mismatch");
    for (std::size_t i = 0; i < 6; ++i) {
        require(near(collective.actuator_commands.values[i], 1.0),
            "Hexa collective output mismatch");
    }

    auto input = make_hexa();
    input.command.torque_z = 3.0;
    const auto yaw = backend.run(input);
    require(yaw.status.clipped, "Large yaw command must clip");
    require(yaw.status.unallocated_torque_z > 0.0,
        "Yaw clipping must report positive unallocated torque");
    for (std::size_t i = 0; i < 6; ++i) {
        require(yaw.actuator_commands.values[i] >= 0.0
                && yaw.actuator_commands.values[i] <= 4.0,
            "Desaturated Hexa output must remain in caller limits");
    }
}

void verifies_missing_axis_authority_is_reported()
{
    adapter::ApControlAllocationBackend backend;
    auto input = make_quadx();
    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].geometry.moment_ratio = 0.0;
    }
    input.command.torque_z = 0.4;
    const auto output = backend.run(input);
    require(output.status.clipped, "Missing yaw authority must report clipping");
    require(near(output.status.unallocated_torque_z, 0.4),
        "Missing yaw authority must remain fully unallocated");
}

void verifies_run_input_limits_trim_and_linearization()
{
    adapter::ApControlAllocationBackend backend;
    auto input = make_quadx();
    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].limit.max = 2.0;
    }
    const auto hover = backend.run(input);
    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        require(near(hover.actuator_commands.values[i], 1.0),
            "Hover output must be derived from the run input maximum");
    }

    input.command = {};
    input.actuators[0].trim = 0.3;
    input.actuators[0].linearization_point = 0.1;
    const auto biased = backend.run(input);
    require(near(biased.actuator_commands.values[0], 0.2),
        "Trim and linearization point must be applied from the run input");
    for (std::size_t i = 1; i < input.actuator_count; ++i) {
        require(near(biased.actuator_commands.values[i], 0.0),
            "Run input affine bias must not affect other actuators");
    }
}

void verifies_ardupilot_463_contract_guards()
{
    adapter::ApControlAllocationBackend backend;
    auto too_many = make_hexa();
    too_many.actuator_count = 13;
    bool rejected_count = false;
    try {
        (void)backend.run(too_many);
    } catch (const std::invalid_argument&) {
        rejected_count = true;
    }
    require(rejected_count, "Copter 4.6.3 motor count above 12 must be rejected");

    auto tilted = make_quadx();
    tilted.actuators[0].geometry.axis = {0.1, 0.0, -1.0};
    bool rejected_axis = false;
    try {
        (void)backend.run(tilted);
    } catch (const std::invalid_argument&) {
        rejected_axis = true;
    }
    require(rejected_axis, "Tilted rotor must be rejected by the first matrix backend");
}

void verifies_desaturation_range_over_command_grid()
{
    adapter::ApControlAllocationBackend backend;
    constexpr double commands[] = {-1.5, -0.5, 0.0, 0.5, 1.5};
    constexpr double throttles[] = {-0.2, 0.0, 1.0, 2.0, 3.0};
    for (double throttle : throttles) {
        for (double roll : commands) {
            for (double pitch : commands) {
                for (double yaw : commands) {
                    auto input = make_hexa(throttle);
                    input.command.torque_x = roll;
                    input.command.torque_y = pitch;
                    input.command.torque_z = yaw;
                    const auto output = backend.run(input);
                    for (std::size_t i = 0; i < output.actuator_commands.count; ++i) {
                        require(output.actuator_commands.values[i] >= -1e-9
                                && output.actuator_commands.values[i] <= 4.0 + 1e-9,
                            "ArduPilot desaturation command escaped caller limits");
                    }
                }
            }
        }
    }
}

}  // namespace

int main()
{
    verifies_collective_and_roll_allocation();
    verifies_hexa_and_yaw_desaturation();
    verifies_missing_axis_authority_is_reported();
    verifies_run_input_limits_trim_and_linearization();
    verifies_ardupilot_463_contract_guards();
    verifies_desaturation_range_over_command_grid();
    std::cout << "ap_control_allocation_backend_smoke: PASS\n";
    return EXIT_SUCCESS;
}
