#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>

using namespace hakoniwa::drone::control_adapter;

int main()
{
    ApEkf3Backend ekf;
    // Exercise the state that cannot be represented by in_air alone: armed
    // on the ground before the simulated takeoff.
    ekf.set_armed_status(true);
    ekf.set_vehicle_at_rest(false);
    ekf.set_in_air_status(false);

    constexpr double dt = 0.0025;
    constexpr std::uint64_t dt_usec = 2500;
    constexpr double latitude = 35.681236;
    constexpr double longitude = 139.767125;
    constexpr double altitude = 40.0;
    constexpr double metres_per_latitude_degree = 111319.49079327358;
    constexpr std::uint64_t stationary_steps = 6000;
    constexpr std::uint64_t total_steps = 16000;

    for (std::uint64_t step = 1; step <= total_steps; ++step) {
        if (step == stationary_steps + 1) {
            ekf.set_in_air_status(true);
        }
        const std::uint64_t time_usec = step * dt_usec;
        const double north_velocity = step > stationary_steps ? 1.0 : 0.0;
        const double travel_time = step > stationary_steps
            ? static_cast<double>(step - stationary_steps) * dt
            : 0.0;
        const double north_position = north_velocity * travel_time;

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
            gps.lat_deg = latitude + north_position / metres_per_latitude_degree;
            gps.lon_deg = longitude;
            gps.alt_m = altitude;
            gps.vel_mps = north_velocity;
            gps.vn_mps = north_velocity;
            gps.satellites_visible = 16;
            gps.eph_m = 0.3;
            gps.epv_m = 0.5;
            gps.sacc_mps = 0.1;
            ekf.push_gps(gps);
        }
        ekf.update();
    }

    const auto state = ekf.get_estimated_state();
    std::cout << "north_m=" << state.position_local_ned_m[0]
              << " north_velocity_mps=" << state.velocity_ned_mps[0]
              << " latitude=" << state.lat_deg
              << " local_position_valid=" << state.local_position_valid
              << " global_position_valid=" << state.global_position_valid << '\n';

    if (!state.attitude_valid
        || !state.local_position_valid
        || !state.global_position_valid) {
        return 1;
    }
    if (state.position_local_ned_m[0] < 20.0
        || state.position_local_ned_m[0] > 30.0) {
        return 2;
    }
    if (state.velocity_ned_mps[0] < 0.7
        || state.velocity_ned_mps[0] > 1.3) {
        return 3;
    }
    if (std::abs(state.position_local_ned_m[1]) > 1.0
        || std::abs(state.velocity_ned_mps[1]) > 0.2) {
        return 4;
    }
    return 0;
}
