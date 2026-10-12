#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hakoniwa::drone::control_adapter;

int main()
{
    ApEkf3Backend ekf;
    ekf.set_debug_csv_path("");
    const auto debug_path = std::filesystem::temp_directory_path() / "hakoniwa-ap-ekf3-debug-test.csv";
    std::filesystem::remove(debug_path);
    ekf.set_debug_csv_path(debug_path.string());
    ekf.set_gps_lag_sec(0.0);
    ekf.set_gps_lag_sec(0.25);
    for (double invalid : {-0.001, 0.251, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected = false;
        try { ekf.set_gps_lag_sec(invalid); }
        catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) { return 9; }
    }
    ekf.set_gps_lag_sec(0.1);
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

    // The production host provides in_air/at_rest but currently has no
    // independent arm-state source.  A climb must nevertheless let Copter's
    // NavEKF3 complete its in-flight yaw/magnetic reset and enter 3-axis fusion.
    ekf.set_in_air_status(true);
    ekf.set_vehicle_at_rest(false);
    constexpr std::uint64_t flight_step_count = 3200;
    for (std::uint64_t offset = 1; offset <= flight_step_count; ++offset) {
        const std::uint64_t step = step_count + offset;
        const std::uint64_t time_usec = step * dt_usec;
        const double climb_m = std::min(4.0, static_cast<double>(offset) * dt / 2.0);
        ekf.push_imu(EkfImuInput{time_usec, 0.0, 0.0, -9.80665, 0.0, 0.0, 0.0}, dt);
        if (step % 8 == 0) {
            ekf.push_mag(EkfMagInput{time_usec, 0.22, 0.0, 0.43});
        }
        if (step % 20 == 0) {
            ekf.push_baro(EkfBaroInput{time_usec, climb_m});
        }
        if (step % 80 == 0) {
            EkfHilGpsInput gps{};
            gps.time_usec = time_usec;
            gps.fix_type = 3;
            gps.lat_deg = latitude;
            gps.lon_deg = longitude;
            gps.alt_m = altitude + climb_m;
            gps.vd_mps = climb_m < 4.0 ? -0.5 : 0.0;
            gps.satellites_visible = 16;
            gps.eph_m = 0.5;
            gps.epv_m = 0.8;
            gps.sacc_mps = 0.2;
            ekf.push_gps(gps);
        }
        ekf.update();
    }

    const auto state = ekf.get_estimated_state();
    ekf.set_debug_csv_path("");
    {
        std::ifstream csv(debug_path);
        std::string header, row;
        if (!std::getline(csv, header) || !std::getline(csv, row)) { return 10; }
        const auto columns = [](const std::string& line) {
            return static_cast<int>(std::count(line.begin(), line.end(), ',')) + 1;
        };
        if (columns(header) != 42 || columns(row) != columns(header)
            || header.rfind("TimeUS,Roll,Pitch,Yaw", 0) != 0) { return 11; }
        bool saw_three_axis_fusion = false;
        do {
            std::vector<std::string> fields;
            std::stringstream stream(row);
            std::string field;
            while (std::getline(stream, field, ',')) {
                fields.push_back(field);
            }
            if (fields.size() == 42 && fields.back() == "1") {
                saw_three_axis_fusion = true;
            }
        } while (std::getline(csv, row));
        if (!saw_three_axis_fusion) { return 12; }
    }
    std::filesystem::remove(debug_path);
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

    if (state.time_usec != (step_count + flight_step_count) * dt_usec
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
        if (!std::isfinite(value)) { return 5; }
    }
    if (std::abs(state.position_local_ned_m[0]) > 2.0
        || std::abs(state.position_local_ned_m[1]) > 2.0
        || state.position_local_ned_m[2] > -2.5
        || state.position_local_ned_m[2] < -6.0) {
        return 5;
    }
    if (std::abs(state.lat_deg - latitude) > 1.0e-4
        || std::abs(state.lon_deg - longitude) > 1.0e-4
        || std::abs(state.alt_m_amsl - (altitude + 4.0)) > 2.0) {
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
    ekf.reset();
    ekf.push_imu(EkfImuInput{dt_usec, 0.0, 0.0, -9.80665, 0.0, 0.0, 0.0}, dt);
    ekf.update();
    if (ekf.get_estimated_state().time_usec != dt_usec) {
        return 8;
    }
    return 0;
}
