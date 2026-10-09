// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_allocation_feedback_policy.hpp"
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iterator>
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
    const double cdss_to_rad=3.14159265358979323846/18000.0;
    assert(std::abs(c.attitude_control.accel_roll_max_rad_sec2 - 110000.0*cdss_to_rad) < 1e-12);
    assert(std::abs(c.attitude_control.accel_pitch_max_rad_sec2 - 110000.0*cdss_to_rad) < 1e-12);
    assert(std::abs(c.attitude_control.accel_yaw_max_rad_sec2 - 27000.0*cdss_to_rad) < 1e-12);
    std::ifstream config_file(argv[1]);
    const std::string config(std::istreambuf_iterator<char>(config_file), {});
    const auto configured=loader.load_from_text(
        config.substr(0, config.find_last_of('}')) +
        ",\n\"ATC_ACCEL_R_MAX\": 12345,\n\"ATC_ACCEL_P_MAX\": 23456,\n\"ATC_ACCEL_Y_MAX\": 34567\n}");
    assert(std::abs(configured.attitude_control.accel_roll_max_rad_sec2 - 12345.0*cdss_to_rad) < 1e-12);
    assert(std::abs(configured.attitude_control.accel_pitch_max_rad_sec2 - 23456.0*cdss_to_rad) < 1e-12);
    assert(std::abs(configured.attitude_control.accel_yaw_max_rad_sec2 - 34567.0*cdss_to_rad) < 1e-12);
    std::cout << "ap_contract_smoke: PASS\n";
}
