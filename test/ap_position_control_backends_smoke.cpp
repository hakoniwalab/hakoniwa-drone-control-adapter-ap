// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace adapter = hakoniwa::drone::control_adapter;

namespace {

constexpr double pi = 3.14159265358979323846;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void altitude_feedback_is_not_jerk_limited()
{
    adapter::ApAltitudeControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.acceleration_p = 0.5;
    config.acceleration_i = config.acceleration_d = config.acceleration_feed_forward = 0.0;
    config.acceleration_target_filter_hz = config.acceleration_error_filter_hz = 0.0;
    config.acceleration_derivative_filter_hz = 0.0;
    config.acceleration_max_mps2 = config.jerk_max_mps3 = 0.01;
    config.hover_thrust = 0.5;
    config.thrust_max = 2.0;
    adapter::ApAltitudeControlBackend backend(config);

    adapter::AltitudeControlInput input{};
    input.mode = adapter::AltitudeControlMode::Velocity;
    input.target_velocity.vz = 2.0;
    const auto output = backend.run(input, 0.01);

    require(output.body_z < -1.1,
        "First-cycle altitude feedback must not be limited to jerk*dt");
}

void altitude_throttle_filter_has_step_response_and_reset()
{
    adapter::ApAltitudeControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.acceleration_p = 1.0;
    config.acceleration_i = config.acceleration_d = config.acceleration_feed_forward = 0.0;
    config.acceleration_target_filter_hz = config.acceleration_error_filter_hz = 0.0;
    config.acceleration_derivative_filter_hz = 0.0;
    config.hover_thrust = 0.5;
    config.thrust_max = 2.0;
    config.throttle_filter_hz = 2.0;
    adapter::ApAltitudeControlBackend backend(config);

    constexpr double dt = 0.01;
    adapter::AltitudeControlInput input{};
    input.mode = adapter::AltitudeControlMode::Velocity;
    require(std::abs(backend.run(input, dt).body_z + 1.0) < 1.0e-12,
        "Throttle filter must start from the first in-air throttle value");

    input.target_velocity.vz = 1.0;
    const double alpha = dt / (dt + 1.0 / (2.0 * pi * config.throttle_filter_hz));
    const auto first_step = backend.run(input, dt);
    require(std::abs(first_step.body_z - (-1.0 - 0.2 * alpha)) < 1.0e-9,
        "First throttle step must move by the 2 Hz low-pass alpha");
    auto settled = first_step;
    for (int i = 0; i < 1000; ++i) {
        settled = backend.run(input, dt);
    }
    require(std::abs(settled.body_z + 1.2) < 1.0e-6,
        "Filtered throttle must converge to the commanded throttle");

    backend.reset();
    require(std::abs(backend.run(input, dt).body_z + 1.2) < 1.0e-12,
        "Reset throttle filter must initialize from the next throttle command");

    config.throttle_filter_hz = 0.0;
    backend.set_config(config);
    input.target_velocity.vz = 0.0;
    (void)backend.run(input, dt);
    input.target_velocity.vz = 1.0;
    require(std::abs(backend.run(input, dt).body_z + 1.2) < 1.0e-12,
        "Zero throttle filter frequency must disable filtering");
}

void horizontal_feedback_is_limited_only_by_lean_angle()
{
    adapter::ApHorizontalPositionControlBackendConfig config{};
    config.velocity_p = 1.0;
    config.velocity_i = config.velocity_d = config.velocity_feed_forward = 0.0;
    config.velocity_error_filter_hz = config.velocity_derivative_filter_hz = 0.0;
    config.speed_max_mps = 100.0;
    config.acceleration_max_mps2 = config.jerk_max_mps3 = 0.01;
    config.angle_max_rad = 0.2;
    adapter::ApHorizontalPositionControlBackend backend(config);

    adapter::HorizontalPositionControlInput input{};
    input.mode = adapter::HorizontalControlMode::Velocity;
    input.target_velocity.vx = 100.0;
    const auto output = backend.run(input, 0.01);

    require(std::abs(output.pitch_rad + config.angle_max_rad) < 1.0e-9,
        "Horizontal feedback must reach the lean-angle limit without jerk limiting");
    require(std::abs(output.roll_rad) < 1.0e-12,
        "Pure north acceleration must not request roll at zero yaw");
}

adapter::Vector3D rotate(const adapter::AttitudeQuaternion& q, const adapter::Vector3D& v)
{
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty),
            v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

void position_3d_uses_target_heading_for_tilt()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.velocity_i = horizontal.velocity_d = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal);

    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 1.0;
    input.target_yaw_rad = pi;
    const auto output = backend.run_position(input, 0.01);
    const auto thrust_direction = rotate(output.target_attitude, {0.0, 0.0, -1.0});
    require(thrust_direction.x > 0.0,
        "With a 180-degree yaw target, world-frame thrust must still point toward +x error");

    backend.reset();
    input.target_yaw_rad.reset();
    input.target_position = {};
    input.state.yaw_rad = 0.7;
    const auto held = backend.run_position(input, 0.01);
    const double yaw = std::atan2(2.0 * (held.target_attitude.w * held.target_attitude.z
                                      + held.target_attitude.x * held.target_attitude.y),
                                  1.0 - 2.0 * (held.target_attitude.y * held.target_attitude.y
                                             + held.target_attitude.z * held.target_attitude.z));
    require(std::abs(yaw - input.state.yaw_rad) < 1.0e-9,
        "Missing yaw target must hold the current yaw");
}

void position_shaping_respects_speed_acceleration_and_stops()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.position_p = 1.0;
    horizontal.velocity_p = 1.0;
    horizontal.velocity_i = horizontal.velocity_d = horizontal.velocity_feed_forward = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    horizontal.speed_max_mps = 5.0;
    horizontal.angle_max_rad = 1.0;
    adapter::ApPositionShapingConfig shaping{true, 2.0, 20.0, 2.5, 20.0};
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping);

    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 30.0;
    constexpr double dt = 0.01;
    double previous_velocity = 0.0;
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    // A unit-mass point vehicle follows the requested pitch acceleration. This
    // exercises the shaper, position loop and stopping trajectory together.
    for (int i = 0; i < 2000; ++i) {
        const auto output = backend.run_position(input, dt);
        const auto& q = output.target_attitude;
        const double pitch = std::asin(std::max(-1.0, std::min(1.0,
            2.0 * (q.w * q.y - q.z * q.x))));
        const double acceleration = -9.80665 * std::tan(pitch);
        input.state.velocity.x += acceleration * dt;
        input.state.position.x += input.state.velocity.x * dt;
        max_velocity = std::max(max_velocity, std::abs(input.state.velocity.x));
        max_acceleration = std::max(max_acceleration,
            std::abs((input.state.velocity.x - previous_velocity) / dt));
        previous_velocity = input.state.velocity.x;
    }
    require(max_velocity <= horizontal.speed_max_mps + 0.08,
        "Shaped 30 m move must not exceed horizontal speed_max_mps");
    require(max_acceleration <= shaping.acceleration_xy_mps2 + 0.08,
        "Shaped 30 m move must not exceed WPNAV_ACCEL in SI units");
    require(std::abs(input.state.position.x - 30.0) < 0.15,
        "Shaped 30 m move must arrive at the target");
    require(std::abs(input.state.velocity.x) < 0.05,
        "Shaped 30 m move must stop at the target");
}

void position_feedback_is_added_after_shaping_speed_limit()
{
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.position_p = 1.0;
    horizontal.velocity_p = 1.0;
    horizontal.velocity_i = horizontal.velocity_d = horizontal.velocity_feed_forward = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    horizontal.speed_max_mps = 5.0;
    horizontal.angle_max_rad = 1.0;
    adapter::ApHorizontalPositionControlBackend backend(horizontal);
    adapter::HorizontalPositionControlInput input{};
    input.mode = adapter::HorizontalControlMode::Position;
    input.target_position.x = 100.0;
    input.target_velocity.vx = horizontal.speed_max_mps;
    (void)backend.run(input, 0.01);
    require(backend.velocity_target_x_mps() > horizontal.speed_max_mps,
        "Position correction must be added after the shaping velocity limit");

    backend.reset();
    input.mode = adapter::HorizontalControlMode::Velocity;
    input.target_velocity.vx = horizontal.speed_max_mps * 2.0;
    (void)backend.run(input, 0.01);
    require(std::abs(backend.velocity_target_x_mps() - horizontal.speed_max_mps) < 1.0e-6,
        "Velocity mode must limit the requested velocity to WPNAV_SPEED");
}

void disabled_position_shaping_passes_target_directly()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.position_p = 1.0;
    horizontal.velocity_p = 1.0;
    horizontal.velocity_i = horizontal.velocity_d = horizontal.velocity_feed_forward = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    horizontal.speed_max_mps = 100.0;
    horizontal.angle_max_rad = 1.0;
    adapter::ApPositionControl3DBackend backend(
        altitude, horizontal, adapter::ApPositionShapingConfig{false});
    adapter::PositionControl3DPositionInput input{};
    input.target_position.x = 30.0;
    const auto output = backend.run_position(input, 0.01);
    const double pitch = std::atan2(
        2.0 * (output.target_attitude.w * output.target_attitude.y
             - output.target_attitude.z * output.target_attitude.x),
        1.0 - 2.0 * (output.target_attitude.x * output.target_attitude.x
                   + output.target_attitude.y * output.target_attitude.y));
    require(std::abs(pitch + horizontal.angle_max_rad) < 1.0e-9,
        "position_input_shaping 0 must hand the full target directly to the position P loop");
}

void flight_phases_spool_land_and_reset()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    altitude.position_p = 1.0;
    altitude.velocity_p = 0.2;
    altitude.velocity_i = altitude.velocity_d = altitude.velocity_feed_forward = 0.0;
    altitude.velocity_filter_hz = altitude.velocity_derivative_filter_hz = 0.0;
    altitude.acceleration_p = 0.2;
    altitude.acceleration_i = 0.1;
    altitude.acceleration_d = altitude.acceleration_feed_forward = 0.0;
    altitude.acceleration_target_filter_hz = altitude.acceleration_error_filter_hz = 0.0;
    altitude.acceleration_derivative_filter_hz = 0.0;
    altitude.hover_thrust = 0.5;
    altitude.throttle_filter_hz = 0.0;
    altitude.speed_up_mps = 6.0;
    altitude.speed_down_mps = 6.0;
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    horizontal.velocity_i = horizontal.velocity_d = 0.0;
    horizontal.velocity_error_filter_hz = horizontal.velocity_derivative_filter_hz = 0.0;
    adapter::ApPositionShapingConfig shaping{true, 2.5, 5.0, 2.5, 5.0};
    adapter::ApFlightPhaseConfig phase{};
    phase.arming_delay_sec = 2.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping, phase);

    adapter::PositionControl3DPositionInput input{};
    input.flight_phase = adapter::FlightPhase::Takeoff;
    input.target_position.z = -5.0;
    constexpr double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        require(std::abs(backend.run_position(input, dt).thrust.body_z) < 1.0e-12,
            "Takeoff arming delay must command zero thrust");
    }
    input.state.position.z = -0.25;
    for (int i = 0; i < 50; ++i) {
        require(std::abs(backend.run_position(input, dt).thrust.body_z) < 1.0e-12,
            "Takeoff spool interval must command zero normalized thrust");
    }
    adapter::PositionControl3DOutput early_slew{};
    for (int i = 0; i < 10; ++i) {
        early_slew = backend.run_position(input, dt);
    }
    // get_throttle_in()/set_throttle_out use normalized thrust before
    // MOT_THST_EXPO and MOT_SPIN_MIN/MAX.  ArduCopter adds dt/TKOFF_SLEW_TIME;
    // TKOFF_THR_MAX is an exit threshold, not a slew-rate multiplier.
    require(std::abs(early_slew.thrust.body_z + 0.1) < 1.0e-9,
        "Takeoff throttle slew must rise at 1/TKOFF_SLEW_TIME in normalized thrust units");

    // ArduCopter does not start the takeoff path merely because the slew has
    // crossed hover.  It keeps land_complete set until motion is detected (or
    // 90% throttle is reached), resetting the z controller on every cycle.
    for (int i = 0; i < 115; ++i) {
        (void)backend.run_position(input, dt);
    }
    const auto above_hover_but_landed = backend.run_position(input, dt);
    require(above_hover_but_landed.thrust.body_z < -1.25,
        "Takeoff must keep slewing throttle after crossing hover while no motion is detected");

    input.target_yaw_rad = pi;
    input.target_yaw_rate_rad_sec = 1.0;
    input.state.velocity.z = -0.7;
    input.state.acceleration.z = -1.2;
    const auto liftoff_detection = backend.run_position(input, dt);
    require(std::abs(liftoff_detection.target_attitude.z) < 1.0e-12 &&
            liftoff_detection.target_yaw_rate_rad_sec.value_or(1.0) == 0.0,
        "Ground takeoff must hold current yaw with zero heading rate");
    const auto first_flying = backend.run_position(input, dt);
    require(first_flying.target_attitude.z > 0.5,
        "Takeoff must accept the guided yaw target on the cycle after liftoff detection");
    require(std::abs(first_flying.thrust.body_z - liftoff_detection.thrust.body_z) < 0.03,
        "First closed-loop altitude output must continue the takeoff throttle slew with measured acceleration");

    input.flight_phase = adapter::FlightPhase::Normal;
    input.target_position = {};
    const auto normal = backend.run_position(input, dt);
    require(normal.thrust.body_z < -0.9,
        "Normal phase must keep the pre-flight-phase position controller behavior");

    auto limits = std::make_shared<adapter::ApMotorThrottleLimits>();
    limits->lower = true;
    backend.set_motor_limits_source(limits);
    input.flight_phase = adapter::FlightPhase::Landing;
    input.state.position.z = 0.0;
    input.state.velocity = {};
    input.state.acceleration = {};
    for (int i = 0; i < 100; ++i) {
        (void)backend.run_position(input, dt);
    }
    require(std::abs(backend.run_position(input, dt).thrust.body_z) < 1.0e-12,
        "Landing detector must stop thrust after one second at the lower limit");

    input.flight_phase = adapter::FlightPhase::Takeoff;
    input.target_position.z = -5.0;
    require(std::abs(backend.run_position(input, dt).thrust.body_z) < 1.0e-12,
        "A new Takeoff phase must reset landed state and restart spool");
}

void landing_descent_velocity_obeys_acceleration_and_jerk_limits()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    altitude.throttle_filter_hz = 0.0;
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    adapter::ApPositionShapingConfig shaping{true, 2.5, 5.0, 1.0, 2.0};
    adapter::ApFlightPhaseConfig phase{};
    phase.land_speed_mps = 0.5;
    phase.land_alt_low_m = 10.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping, phase);

    adapter::PositionControl3DPositionInput input{};
    input.flight_phase = adapter::FlightPhase::Landing;
    input.state.position.z = -5.0;
    constexpr double dt = 0.01;
    double previous_velocity = 0.0;
    double previous_acceleration = 0.0;
    for (int i = 0; i < 300; ++i) {
        (void)backend.run_position(input, dt);
        const double velocity = backend.landing_velocity_target_up_mps();
        const double acceleration = backend.landing_acceleration_target_up_mps2();
        require(std::abs(acceleration - previous_acceleration) <= shaping.jerk_z_mps3 * dt + 1.0e-5,
            "Landing acceleration target must obey PSC_JERK_Z");
        require(std::abs(acceleration) <= shaping.acceleration_z_mps2 + 1.0e-5,
            "Landing acceleration target must obey WPNAV_ACCEL_Z");
        require(std::abs(velocity - (previous_velocity + previous_acceleration * dt)) < 1.0e-5,
            "Landing velocity target must integrate the jerk-limited acceleration");
        previous_velocity = velocity;
        previous_acceleration = acceleration;
    }
    require(std::abs(backend.landing_velocity_target_up_mps() + phase.land_speed_mps) < 1.0e-2,
        "Landing velocity target must settle at LAND_SPEED");
}

void ground_motor_duty_uses_rotor_model_without_changing_flight_thrust()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    altitude.hover_thrust = 0.5;
    altitude.throttle_filter_hz = 0.0;
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    adapter::ApPositionShapingConfig shaping{false};
    adapter::ApFlightPhaseConfig phase{};
    phase.arming_delay_sec = 0.02;
    phase.motor_spool_time_sec = 0.10;
    phase.ground_motor_output = true;
    phase.motor_spin_arm = 0.10;
    phase.motor_spin_min = 0.15;
    phase.motor_spin_max = 0.95;
    phase.motor_thrust_expo = 0.5;
    // omega = voltage*duty/b = 10*duty; hover omega=5, hence
    // T/T_hover=(2*duty)^2.
    phase.motor_model_a_coef = 0.0;
    phase.motor_model_b_coef = 1.0;
    phase.motor_nominal_voltage = 10.0;
    phase.motor_hover_omega = 5.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping, phase);

    adapter::PositionControl3DPositionInput input{};
    input.flight_phase = adapter::FlightPhase::Takeoff;
    input.target_position.z = -5.0;
    constexpr double dt = 0.01;
    require(std::abs(backend.run_position(input, dt).thrust.body_z) < 1.0e-12,
        "Interlock delay must remain SHUT_DOWN even with ground motor output enabled");
    (void)backend.run_position(input, dt);
    adapter::PositionControl3DOutput spool{};
    for (int i = 0; i < 10; ++i) {
        spool = backend.run_position(input, dt);
    }
    require(std::abs(spool.thrust.body_z + 0.09) < 1.0e-9,
        "End of GROUND_IDLE spool must equal rotor thrust at MOT_SPIN_MIN");

    // throttle=0.005 -> curve=(-0.5+sqrt(0.26)), actuator=0.15+0.8*curve,
    // then the rotor model converts Duty to T/T_hover.
    const auto first_flight_thrust = backend.run_position(input, dt);
    const double curve = -0.5 + std::sqrt(0.26);
    const double duty = 0.15 + 0.8 * curve;
    require(std::abs(first_flight_thrust.thrust.body_z + 4.0 * duty * duty) < 1.0e-6,
        "Takeoff slew must pass through MOT_THST_EXPO, spin range and rotor model");

    auto limits = std::make_shared<adapter::ApMotorThrottleLimits>();
    limits->lower = true;
    backend.set_motor_limits_source(limits);
    input.flight_phase = adapter::FlightPhase::Landing;
    input.state = {};
    for (int i = 0; i < 100; ++i) {
        (void)backend.run_position(input, dt);
    }
    const auto ground_idle = backend.run_position(input, dt);
    require(std::abs(ground_idle.thrust.body_z + 0.04) < 1.0e-9,
        "Landed GROUND_IDLE must equal rotor thrust at MOT_SPIN_ARM");
}

void takeoff_slew_uses_ardupilot_scheduler_dt()
{
    adapter::ApAltitudeControlBackendConfig altitude{};
    altitude.hover_thrust = 0.5;
    altitude.throttle_filter_hz = 0.0;
    adapter::ApHorizontalPositionControlBackendConfig horizontal{};
    adapter::ApPositionShapingConfig shaping{false};
    adapter::ApFlightPhaseConfig phase{};
    phase.arming_delay_sec = 0.003;
    phase.motor_spool_time_sec = 0.003;
    phase.takeoff_slew_time_sec = 2.0;
    phase.takeoff_slew_loop_rate_hz = 400.0;
    adapter::ApPositionControl3DBackend backend(altitude, horizontal, shaping, phase);

    adapter::PositionControl3DPositionInput input{};
    input.flight_phase = adapter::FlightPhase::Takeoff;
    input.target_position.z = -5.0;
    constexpr double actual_dt = 0.003;
    (void)backend.run_position(input, actual_dt); // ARMING_DELAY
    (void)backend.run_position(input, actual_dt); // GROUND_IDLE reaches spin-min
    const auto first = backend.run_position(input, actual_dt);
    const auto second = backend.run_position(input, actual_dt);
    require(std::abs(first.thrust.body_z + 0.0025) < 1.0e-12 &&
            std::abs(second.thrust.body_z + 0.0050) < 1.0e-12,
        "Takeoff slew must add G_Dt/TKOFF_SLEW_TIME, not elapsed simulation dt");
}


}  // namespace

int main()
{
    altitude_feedback_is_not_jerk_limited();
    altitude_throttle_filter_has_step_response_and_reset();
    horizontal_feedback_is_limited_only_by_lean_angle();
    position_3d_uses_target_heading_for_tilt();
    position_shaping_respects_speed_acceleration_and_stops();
    position_feedback_is_added_after_shaping_speed_limit();
    disabled_position_shaping_passes_target_directly();
    flight_phases_spool_land_and_reset();
    landing_descent_velocity_obeys_acceleration_and_jerk_limits();
    ground_motor_duty_uses_rotor_model_without_changing_flight_thrust();
    takeoff_slew_uses_ardupilot_scheduler_dt();
    std::cout << "ap_position_control_backends_smoke: PASS\n";
    return 0;
}
