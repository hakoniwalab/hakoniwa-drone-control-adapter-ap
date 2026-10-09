// SPDX-License-Identifier: GPL-3.0-or-later

#include "ardupilot_fixture.hpp"

#include "hakoniwa/drone/control_adapter/ap_allocation_feedback_policy.hpp"
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_attitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_control_allocation_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_rate_control_backend.hpp"

namespace hako::control_link::contract {
namespace adapter = hakoniwa::drone::control_adapter;
namespace {

adapter::ApAltitudeControlBackendConfig altitude_config(FixtureProfile profile)
{
    adapter::ApAltitudeControlBackendConfig config{};
    config.velocity_p = profile == FixtureProfile::IntegralOnly ? 0.0 : 5.0;
    config.velocity_i = profile == FixtureProfile::IntegralOnly ? 1.0 : 0.0;
    config.velocity_d = 0.0;
    config.velocity_feed_forward = 0.0;
    config.acceleration_p = 0.0;
    config.acceleration_i = 0.0;
    config.acceleration_d = 0.0;
    config.acceleration_feed_forward = 0.001;
    config.hover_thrust = 1.0 / kContractActuatorLimit;
    return config;
}

adapter::ApHorizontalPositionControlBackendConfig horizontal_config(FixtureProfile profile)
{
    adapter::ApHorizontalPositionControlBackendConfig config{};
    config.velocity_p = profile == FixtureProfile::IntegralOnly ? 0.0 : 2.0;
    config.velocity_i = profile == FixtureProfile::IntegralOnly ? 1.0 : 0.2;
    config.velocity_d = 0.0;
    config.velocity_feed_forward = 0.0;
    return config;
}

adapter::ApRateControlBackendConfig rate_config(FixtureProfile profile)
{
    adapter::ApRateControlBackendConfig config{};
    auto axis = adapter::ApRateControlAxisConfig{};
    axis.p = profile == FixtureProfile::IntegralOnly ? 0.0 : 0.135;
    axis.i = profile == FixtureProfile::IntegralOnly ? 1.0 : 0.135;
    axis.d = 0.0;
    axis.feed_forward = 0.0;
    axis.integrator_limit = 10.0;
    config.roll = axis;
    config.pitch = axis;
    config.yaw = axis;
    return config;
}

}  // namespace

std::unique_ptr<adapter::IAltitudeControlBackend>
ArduPilotFixture::create_altitude(FixtureProfile profile) const
{
    return std::make_unique<adapter::ApAltitudeControlBackend>(altitude_config(profile));
}

std::unique_ptr<adapter::IHorizontalPositionControlBackend>
ArduPilotFixture::create_horizontal(FixtureProfile profile) const
{
    return std::make_unique<adapter::ApHorizontalPositionControlBackend>(horizontal_config(profile));
}

std::unique_ptr<adapter::IPositionControl3DBackend>
ArduPilotFixture::create_position_3d(FixtureProfile) const
{
    return nullptr;
}

std::unique_ptr<adapter::IAttitudeControlBackend>
ArduPilotFixture::create_attitude(FixtureProfile profile) const
{
    if (profile == FixtureProfile::IntegralOnly) return nullptr;
    return std::make_unique<adapter::ApAttitudeControlBackend>();
}

std::unique_ptr<adapter::IRateControlBackend>
ArduPilotFixture::create_rate(FixtureProfile profile) const
{
    return std::make_unique<adapter::ApRateControlBackend>(rate_config(profile));
}

std::unique_ptr<adapter::IControlAllocationBackend>
ArduPilotFixture::create_allocation(FixtureProfile) const
{
    adapter::ApControlAllocationBackendConfig config{};
    return std::make_unique<adapter::ApControlAllocationBackend>(config);
}

std::unique_ptr<adapter::IAllocationFeedbackPolicy>
ArduPilotFixture::create_feedback(FixtureProfile) const
{
    return std::make_unique<adapter::ApAllocationFeedbackPolicy>();
}

std::unique_ptr<adapter::IEkfAdapter>
ArduPilotFixture::create_ekf(FixtureProfile) const
{
    return nullptr;
}

adapter::ControlAllocationInput ArduPilotFixture::geometry() const
{
    adapter::ControlAllocationInput input{};
    input.actuator_count = 4;
    const double positions[4][2] = {
        {0.2, -0.2}, {0.2, 0.2}, {-0.2, 0.2}, {-0.2, -0.2},
    };
    for (std::size_t i = 0; i < input.actuator_count; ++i) {
        input.actuators[i].geometry.position = {positions[i][0], positions[i][1], 0.0};
        input.actuators[i].geometry.axis = {0.0, 0.0, -1.0};
        input.actuators[i].geometry.thrust_coefficient = kContractCt;
        input.actuators[i].geometry.moment_ratio =
            (i % 2 == 0 ? -1.0 : 1.0) * kContractMomentRatio;
        input.actuators[i].limit = {0.0, kContractActuatorLimit};
    }
    return input;
}

}  // namespace hako::control_link::contract
