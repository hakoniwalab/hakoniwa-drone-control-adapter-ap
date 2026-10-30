// SPDX-License-Identifier: GPL-3.0-or-later
// [SEN-COM-350] The mount parameters reach NavEKF3 through ApEkf3Backend (orientation, INS_POS1, GPS1_POS).
#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>
#include "hakoniwa/drone/control_adapter/ap_sensor_model_v2.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

namespace {
constexpr double kLatitude = 35.681236;
constexpr double kLongitude = 139.767125;

// Body-frame inputs at rest, heading north; sensor values are body values turned back to the sensor axes.
// spin_imu_x_m > 0 adds a 1 rad/s yaw spin after alignment with the IMU that far ahead of the origin; returns the
// largest horizontal speed estimated for the (stationary) origin after the first 10 s of the spin through max_speed_mps.
EkfEstimatedState run_at_rest(const ArdupilotSensorModelV2Config& mount, bool upside_down,
    double spin_imu_x_m = 0.0, double* max_speed_mps = nullptr, std::array<double, 3> mag_bias_gauss = {})
{
    ApEkf3Backend ekf;
    ekf.set_debug_csv_path("");
    ekf.set_gps_lag_sec(0.0);
    ekf.set_sensor_model_v2_config(mount);
    EkfAdapterConfig config{};
    config.mag_declination_deg = 0.0;
    ekf.set_config(config);
    ekf.set_armed_status(false);
    ekf.set_vehicle_at_rest(true);
    ekf.set_in_air_status(false);
    // ROTATION_ROLL_180 maps sensor (x, y, z) to body (x, -y, -z).
    const double flip = upside_down ? -1.0 : 1.0;
    constexpr double dt = 0.0025;
    constexpr std::uint64_t dt_usec = 2500;
    constexpr std::uint64_t align_steps = 24000;
    const std::uint64_t steps = spin_imu_x_m > 0.0 ? align_steps + 8000 : align_steps;
    double yaw = 0.0;
    for (std::uint64_t step = 1; step <= steps; ++step) {
        const std::uint64_t time_usec = step * dt_usec;
        const bool spinning = step > align_steps;
        if (spinning && step == align_steps + 1) { ekf.set_vehicle_at_rest(false); }
        // The IMU circles the origin: its specific force has the centripetal -w^2 r along body x.
        const double rate = spinning ? 1.0 : 0.0;
        const double centripetal = spinning ? -spin_imu_x_m : 0.0;
        ekf.push_imu(EkfImuInput{time_usec, centripetal, 0.0, flip * -9.80665, 0.0, 0.0, flip * rate}, dt);
        yaw += rate * dt;
        if (step % 8 == 0) {
            ekf.push_mag(EkfMagInput{time_usec, 0.22 * std::cos(yaw) + mag_bias_gauss[0],
                flip * -0.22 * std::sin(yaw) + mag_bias_gauss[1], flip * 0.43 + mag_bias_gauss[2]});
        }
        if (step % 20 == 0) { ekf.push_baro(EkfBaroInput{time_usec, 0.0}); }
        if (step % 80 == 0) {
            EkfHilGpsInput gps{};
            gps.time_usec = time_usec;
            gps.fix_type = 3;
            gps.lat_deg = kLatitude;
            gps.lon_deg = kLongitude;
            gps.alt_m = 40.0;
            gps.satellites_visible = 16;
            gps.eph_m = 0.5;
            gps.epv_m = 0.8;
            gps.sacc_mps = 0.2;
            ekf.push_gps(gps);
        }
        ekf.update();
        if (step > align_steps + 4000 && max_speed_mps != nullptr) {
            const auto state = ekf.get_estimated_state();
            *max_speed_mps = std::fmax(*max_speed_mps, std::hypot(state.velocity_ned_mps[0], state.velocity_ned_mps[1]));
        }
    }
    return ekf.get_estimated_state();
}

double tilt_deg(const EkfEstimatedState& state)
{
    const auto& q = state.attitude_quaternion_wxyz;
    // Angle between the estimated body z axis and the local down axis.
    const double down_z = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    return std::acos(std::fmax(-1.0, std::fmin(1.0, down_z))) * 180.0 / M_PI;
}

double yaw_deg(const EkfEstimatedState& state)
{
    const auto& q = state.attitude_quaternion_wxyz;
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) * 180.0 / M_PI;
}

double north_m(const EkfEstimatedState& state)
{
    return (state.lat_deg - kLatitude) * M_PI / 180.0 * 6378137.0;
}

int fail(const char* message)
{
    std::cerr << "ap_sensor_model_v2_ekf_smoke: " << message << '\n';
    return EXIT_FAILURE;
}
}

int main()
{
    const auto baseline = run_at_rest({}, false);
    ArdupilotSensorModelV2Config flipped{};
    flipped.board_orientation = 8;
    const auto rotated = run_at_rest(flipped, true);
    ArdupilotSensorModelV2Config antenna{};
    antenna.gps_position_body_m = {0.5, 0.0, 0.0};
    const auto gps_forward = run_at_rest(antenna, false);
    ArdupilotSensorModelV2Config imu{};
    imu.imu_position_body_m = {0.5, 0.0, 0.0};
    double speed_with_offset = 0.0;
    double speed_without_offset = 0.0;
    run_at_rest(imu, false, 0.5, &speed_with_offset);
    run_at_rest({}, false, 0.5, &speed_without_offset);
    // AP_Compass corrects S * (m + COMPASS_OFS); a raw field biased by -COMPASS_OFS must give the baseline yaw.
    ArdupilotSensorModelV2Config compass{};
    compass.mag_offset_milligauss = {-30.0, 15.0, -8.0};
    const std::array<double, 3> bias{0.030, -0.015, 0.008};
    const auto calibrated = run_at_rest(compass, false, 0.0, nullptr, bias);
    const auto uncalibrated = run_at_rest({}, false, 0.0, nullptr, bias);
    std::cout << "yaw baseline=" << yaw_deg(baseline) << " calibrated=" << yaw_deg(calibrated)
              << " uncalibrated=" << yaw_deg(uncalibrated) << '\n';
    std::cout << "tilt baseline=" << tilt_deg(baseline) << " rotated=" << tilt_deg(rotated)
              << " north baseline=" << north_m(baseline) << " gps_forward=" << north_m(gps_forward)
              << " spin speed with INS_POS1=" << speed_with_offset << " without=" << speed_without_offset << '\n';
    if (!baseline.attitude_valid || !rotated.attitude_valid) { return fail("attitude is not valid"); }
    if (tilt_deg(rotated) > 1.0) { return fail("AHRS_ORIENTATION did not bring the upside-down IMU back to level"); }
    // The estimate is the IMU position; the antenna 0.5 m ahead of it puts the IMU 0.5 m behind the fix.
    if (std::abs(north_m(gps_forward) - north_m(baseline) + 0.5) > 0.1) { return fail("GPS1_POS did not reach EKF"); }
    // At rest the origin output cancels INS_POS1; a spin shows it: the origin stays still only when the lever arm is known.
    if (speed_with_offset > 0.1 || speed_without_offset < 0.3) { return fail("INS_POS1 did not reach EKF"); }
    if (std::abs(yaw_deg(calibrated) - yaw_deg(baseline)) > 0.5) { return fail("COMPASS_OFS did not reach EKF"); }
    if (std::abs(yaw_deg(uncalibrated) - yaw_deg(baseline)) < 1.5) { return fail("the compass bias case is too weak"); }
    std::cout << "ap_sensor_model_v2_ekf_smoke: PASS\n";
    return EXIT_SUCCESS;
}
