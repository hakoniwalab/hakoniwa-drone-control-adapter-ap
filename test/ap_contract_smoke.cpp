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
#include <regex>
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
    assert(c.ekf.gps_delay_ms >= 0.0 && c.ekf.gps_delay_ms <= 250.0);
    assert(c.ekf.hgt_delay_ms == 60.0);
    assert(c.rate_control.roll.p > 0 && c.rate_control.pitch.p > 0 && c.rate_control.yaw.p > 0);
    const double cdss_to_rad=3.14159265358979323846/18000.0;
    assert(std::abs(c.attitude_control.accel_roll_max_rad_sec2 - 110000.0*cdss_to_rad) < 1e-12);
    assert(std::abs(c.attitude_control.accel_pitch_max_rad_sec2 - 110000.0*cdss_to_rad) < 1e-12);
    assert(std::abs(c.attitude_control.accel_yaw_max_rad_sec2 - 27000.0*cdss_to_rad) < 1e-12);
    assert(!c.attitude_control.rate_feedforward_enabled);
    assert(c.altitude_control.throttle_filter_hz == 0.0);
    std::ifstream config_file(argv[1]);
    const std::string config(std::istreambuf_iterator<char>(config_file), {});
    const auto configured=loader.load_from_text(
        config.substr(0, config.find_last_of('}')) +
        ",\n\"ATC_ACCEL_R_MAX\": 12345,\n\"ATC_ACCEL_P_MAX\": 23456,\n\"ATC_ACCEL_Y_MAX\": 34567\n}");
    assert(std::abs(configured.attitude_control.accel_roll_max_rad_sec2 - 12345.0*cdss_to_rad) < 1e-12);
    assert(std::abs(configured.attitude_control.accel_pitch_max_rad_sec2 - 23456.0*cdss_to_rad) < 1e-12);
    assert(std::abs(configured.attitude_control.accel_yaw_max_rad_sec2 - 34567.0*cdss_to_rad) < 1e-12);
    std::string without_optional = std::regex_replace(
        config, std::regex("\\\"throttle_filter_hz\\\"\\s*:\\s*0(?:\\.0)?\\s*,"), "");
    without_optional = std::regex_replace(
        without_optional, std::regex(",?\\s*\\\"ATC_RATE_FF_ENAB\\\"\\s*:\\s*0"), "");
    const auto defaults = loader.load_from_text(without_optional);
    assert(defaults.attitude_control.rate_feedforward_enabled);
    assert(std::abs(defaults.attitude_control.input_time_constant_sec - 0.15) < 1e-12);
    assert(std::abs(defaults.attitude_control.slew_yaw_rad_sec - 6000.0*cdss_to_rad) < 1e-12);
    assert(std::abs(defaults.altitude_control.throttle_filter_hz - 2.0) < 1e-12);
    std::string without_delays = std::regex_replace(
        config, std::regex(",?\\s*\\\"GPS1_DELAY_MS\\\"\\s*:\\s*[-+0-9.eE]+"), "");
    without_delays = std::regex_replace(
        without_delays, std::regex(",?\\s*\\\"EK3_HGT_DELAY\\\"\\s*:\\s*[-+0-9.eE]+"), "");
    const auto delay_defaults = loader.load_from_text(without_delays);
    assert(delay_defaults.ekf.gps_delay_ms == 0.0);
    assert(delay_defaults.ekf.hgt_delay_ms == 60.0);
    const auto with_delays = loader.load_from_text(
        without_delays.substr(0, without_delays.find_last_of('}')) +
        ",\n\"GPS1_DELAY_MS\": 250,\n\"EK3_HGT_DELAY\": 60\n}");
    assert(with_delays.ekf.gps_delay_ms == 250.0);
    const auto with_position_limits = loader.load_from_text(
        config.substr(0, config.find_last_of('}')) +
        ",\n\"position_input_shaping\": 1,\n\"WPNAV_ACCEL\": 1.75,\n\"WPNAV_ACCEL_Z\": 3.25\n}");
    assert(std::abs(with_position_limits.position_shaping.acceleration_xy_mps2 - 1.75) < 1e-12);
    assert(std::abs(with_position_limits.position_shaping.acceleration_z_mps2 - 3.25) < 1e-12);
    auto omitted_position_accel = with_position_limits;
    omitted_position_accel.position_shaping.acceleration_xy_mps2 = 0.0;
    const auto derived_position_limits = position_shaping_for(omitted_position_accel);
    assert(std::abs(derived_position_limits.acceleration_xy_mps2
        - 0.5 * 9.80665 * std::tan(omitted_position_accel.horizontal_control.angle_max_rad)) < 1e-12);
    auto rejects_delay = [&](const char* key, const char* value) {
        bool rejected = false;
        try {
            loader.load_from_text(without_delays.substr(0, without_delays.find_last_of('}')) +
                ",\n\"" + key + "\": " + value + "\n}");
        } catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    };
    rejects_delay("GPS1_DELAY_MS", "-1");
    rejects_delay("GPS1_DELAY_MS", "251");
    rejects_delay("EK3_HGT_DELAY", "59");
    std::cout << "ap_contract_smoke: PASS\n";
}
