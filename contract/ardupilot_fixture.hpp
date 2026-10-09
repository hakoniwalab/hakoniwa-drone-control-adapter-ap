#pragma once

// SPDX-License-Identifier: GPL-3.0-or-later

#include "test/contract/fixtures/adapter_fixture.hpp"

namespace hako::control_link::contract {

class ArduPilotFixture final : public AdapterFixture {
public:
    std::string name() const override { return "ardupilot"; }

    std::unique_ptr<hakoniwa::drone::control_adapter::IAltitudeControlBackend>
        create_altitude(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IHorizontalPositionControlBackend>
        create_horizontal(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IPositionControl3DBackend>
        create_position_3d(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IAttitudeControlBackend>
        create_attitude(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IRateControlBackend>
        create_rate(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IControlAllocationBackend>
        create_allocation(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IAllocationFeedbackPolicy>
        create_feedback(FixtureProfile profile) const override;
    std::unique_ptr<hakoniwa::drone::control_adapter::IEkfAdapter>
        create_ekf(FixtureProfile profile) const override;
    hakoniwa::drone::control_adapter::ControlAllocationInput geometry() const override;
};

}  // namespace hako::control_link::contract
