// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_rate_control_backend.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace adapter = hakoniwa::drone::control_adapter;

namespace {

bool near(double actual, double expected, double tolerance = 1e-6)
{
    return std::abs(actual - expected) <= tolerance;
}

void verifies_axis_mapping_and_proportional_control()
{
    adapter::ApRateControlBackendConfig config{};
    config.roll.p = 0.2;
    config.pitch.p = 0.3;
    config.yaw.p = 0.4;

    adapter::ApRateControlBackend backend(config);
    adapter::RateControlInput input{};
    input.dt_sec = 0.01;
    input.target = {1.0, -2.0, 0.5};

    const auto output = backend.run(input);
    assert(near(output.x, 0.2));
    assert(near(output.y, -0.6));
    assert(near(output.z, 0.2));
}

void verifies_feed_forward_matches_ardupilot_motor_input_sum()
{
    adapter::ApRateControlBackendConfig config{};
    config.roll.feed_forward = 0.1;

    adapter::ApRateControlBackend backend(config);
    adapter::RateControlInput input{};
    input.dt_sec = 0.01;
    input.target.p = 2.0;

    const auto output = backend.run(input);
    assert(near(output.x, 0.2));
    assert(near(backend.get_status().roll.feed_forward, 0.2));
}

void verifies_saturation_prevents_integrator_growth()
{
    adapter::ApRateControlBackendConfig config{};
    config.roll.i = 1.0;
    config.roll.integrator_limit = 1.0;

    adapter::ApRateControlBackend backend(config);
    adapter::RateControlInput input{};
    input.dt_sec = 0.1;
    input.target.p = 1.0;

    const auto first = backend.run(input);
    assert(near(first.x, 0.1));

    input.saturation.roll.positive = true;
    const auto limited = backend.run(input);
    assert(near(limited.x, 0.1));
}

void verifies_landed_resets_integrator()
{
    adapter::ApRateControlBackendConfig config{};
    config.roll.p = 0.0; config.roll.i = 1.0; config.roll.integrator_limit = 1.0;
    adapter::ApRateControlBackend backend(config);
    adapter::RateControlInput input{}; input.dt_sec = 0.1; input.target.p = 1.0;
    assert(backend.run(input).x > 0.0);
    input.landed = true; (void)backend.run(input);
    input.landed = false; input.target.p = 0.0;
    assert(near(backend.run(input).x, 0.0));
}

void verifies_previous_rates_setpoint_option()
{
    adapter::ApRateControlBackendConfig delayed_config{};
    delayed_config.roll = {};
    delayed_config.roll.p = 1.0;
    delayed_config.roll.i = 0.0;
    delayed_config.roll.d = 0.0;
    delayed_config.previous_rates_setpoint = true;
    adapter::ApRateControlBackend delayed(delayed_config);
    adapter::RateControlInput input{};
    input.dt_sec = 0.01;
    input.target.p = 1.0;
    assert(near(delayed.run(input).x, 1.0));
    input.target.p = 2.0;
    assert(near(delayed.run(input).x, 1.0));
    delayed.reset();
    input.target.p = 3.0;
    assert(near(delayed.run(input).x, 3.0));

    delayed_config.previous_rates_setpoint = false;
    adapter::ApRateControlBackend current(delayed_config);
    input.target.p = 1.0;
    assert(near(current.run(input).x, 1.0));
    input.target.p = 2.0;
    assert(near(current.run(input).x, 2.0));
}

}  // namespace

int main()
{
    verifies_axis_mapping_and_proportional_control();
    verifies_feed_forward_matches_ardupilot_motor_input_sum();
    verifies_saturation_prevents_integrator_growth();
    verifies_landed_resets_integrator();
    verifies_previous_rates_setpoint_option();
    std::cout << "ap_rate_control_backend_smoke: PASS\n";
    return 0;
}
