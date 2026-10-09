// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_allocation_feedback_policy.hpp"
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace hakoniwa::drone::control_adapter;
int main(int argc, char** argv) {
    ApAltitudeControlBackend altitude; AltitudeControlInput z{}; z.position.z=z.target_altitude=2;
    assert(std::abs(altitude.run(z, .02).body_z + 1) < 1e-12);
    z.target_altitude=3; assert(altitude.run(z,.02).body_z < -1);
    altitude.reset();
    ApHorizontalPositionControlBackend horizontal; HorizontalPositionControlInput xy{};
    xy.mode=HorizontalControlMode::Velocity; xy.target_velocity.vx=1;
    assert(horizontal.run(xy,.02).pitch_rad > 0); horizontal.reset();
    ApAllocationFeedbackPolicy feedback; AllocationFeedbackPolicyInput f{};
    f.allocation_status.unallocated_torque_x=.1; assert(feedback.run(f).roll.positive); feedback.reset();
    assert(argc == 2); ArdupilotControllerConfigLoader loader;
    const auto c=loader.load_from_file(argv[1]); assert(c.altitude_control.hover_thrust == .35);
    assert(c.rate_control.roll.p > 0 && c.rate_control.pitch.p > 0 && c.rate_control.yaw.p > 0);
    std::cout << "ap_contract_smoke: PASS\n";
}
