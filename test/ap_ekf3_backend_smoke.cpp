#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

using namespace hakoniwa::drone::control_adapter;

int main()
{
    ApEkf3Backend ekf;
    EkfAdapterConfig config{};
    config.mag_declination_deg = 0.0;
    ekf.set_config(config);
    ekf.set_armed_status(false);
    ekf.set_vehicle_at_rest(true);
    ekf.set_in_air_status(false);

    constexpr double dt = 0.0025;
    constexpr std::uint64_t dt_usec = 2500;
    constexpr double latitude = 35.681236;
    constexpr double longitude = 139.767125;
    constexpr double altitude = 40.0;

    constexpr std::uint64_t step_count = 24000;
    for (std::uint64_t step = 1; step <= step_count; ++step) {
        const std::uint64_t time_usec = step * dt_usec;
        ekf.push_imu(
            EkfImuInput{time_usec, 0.0, 0.0, -9.80665, 0.0, 0.0, 0.0},
            dt);

        if (step % 8 == 0) {
            ekf.push_mag(EkfMagInput{time_usec, 0.22, 0.0, 0.43});
        }
        if (step % 20 == 0) {
            ekf.push_baro(EkfBaroInput{time_usec, 0.0});
        }
        if (step % 80 == 0) {
            EkfHilGpsInput gps{};
            gps.time_usec = time_usec;
            gps.fix_type = 3;
            gps.lat_deg = latitude;
            gps.lon_deg = longitude;
            gps.alt_m = altitude;
            gps.satellites_visible = 16;
            gps.eph_m = 0.5;
            gps.epv_m = 0.8;
            gps.sacc_mps = 0.2;
            ekf.push_gps(gps);
        }
        ekf.update();
    }

    const auto state = ekf.get_estimated_state();
    std::cout << "time_usec=" << state.time_usec
              << " attitude_valid=" << state.attitude_valid
              << " local_position_valid=" << state.local_position_valid
              << " global_position_valid=" << state.global_position_valid
              << " q=[" << state.attitude_quaternion_wxyz[0] << ','
              << state.attitude_quaternion_wxyz[1] << ','
              << state.attitude_quaternion_wxyz[2] << ','
              << state.attitude_quaternion_wxyz[3] << "]"
              << " velocity=[" << state.velocity_ned_mps[0] << ','
              << state.velocity_ned_mps[1] << ','
              << state.velocity_ned_mps[2] << "]\n";

    if (state.time_usec != step_count * dt_usec
        || !state.attitude_valid
        || !state.local_position_valid
        || !state.global_position_valid) {
        return 1;
    }
    for (double value : state.attitude_quaternion_wxyz) {
        if (!std::isfinite(value)) {
            return 2;
        }
    }
    double quaternion_norm_squared = 0.0;
    for (double value : state.attitude_quaternion_wxyz) {
        quaternion_norm_squared += value * value;
    }
    if (std::abs(quaternion_norm_squared - 1.0) > 1.0e-3) {
        return 4;
    }
    for (double value : state.velocity_ned_mps) {
        if (!std::isfinite(value) || std::abs(value) > 1.0) {
            return 3;
        }
    }
    for (double value : state.position_local_ned_m) {
        if (!std::isfinite(value) || std::abs(value) > 2.0) {
            return 5;
        }
    }
    if (std::abs(state.lat_deg - latitude) > 1.0e-4
        || std::abs(state.lon_deg - longitude) > 1.0e-4
        || std::abs(state.alt_m_amsl - altitude) > 2.0) {
        return 6;
    }

    bool rejected_stale_imu = false;
    try {
        ekf.push_imu(
            EkfImuInput{state.time_usec, 0.0, 0.0, -9.80665, 0.0, 0.0, 0.0},
            dt);
    }
    catch (const std::invalid_argument&) {
        rejected_stale_imu = true;
    }
    if (!rejected_stale_imu) {
        return 7;
    }
    return 0;
}
