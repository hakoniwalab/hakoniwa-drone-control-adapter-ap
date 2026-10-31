// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_sensor_model_v2.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace hakoniwa::drone::control_adapter;

namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void close(const std::array<double, 3>& actual, const std::array<double, 3>& expected, const char* message)
{
    for (unsigned i = 0; i < 3; ++i) require(std::abs(actual[i] - expected[i]) < 1.0e-5, message);
}
std::string with(const std::string& base, const std::string& fields)
{
    const auto end = base.rfind('}');
    require(end != std::string::npos, "invalid fixture JSON");
    return base.substr(0, end) + "," + fields + base.substr(end);
}
}

int main(int argc, char** argv)
{
    require(argc == 2, "expected controller config fixture");
    std::ifstream stream(argv[1]);
    const std::string base{std::istreambuf_iterator<char>(stream), {}};
    ArdupilotSensorModelV2Config yaw{};
    yaw.board_orientation = 2;
    close(ap_rotate_sensor_to_body(yaw, {0.0, -1.0, 0.0}), {1.0, 0.0, 0.0}, "standard Rotation failed");
    // T-ACC-U1-050 / T-MAG-U1-070: the values the adapter passes to EKF3 with AHRS_ORIENTATION = 2 (Yaw 90).
    // The accelerometer keeps the lever arm (INS_POS1 lets EKF3 account for it); 50,000 nT = 0.5 gauss.
    close(ap_rotate_sensor_to_body(yaw, {0.0, 1.3708, -9.81}), {-1.3708, 0.0, -9.81},
        "T-ACC-U1-050: the accelerometer was not only turned to the body axes");
    close(ap_correct_mag_sensor_to_body(yaw, {0.0, -0.5, 0.0}), {0.5, 0.0, 0.0},
        "T-MAG-U1-070: the compass was not turned to the body axes");
    // T-ACC-U1-040: INS_ACCOFFS / INS_ACCSCAL made from fixedMg [20, -15, 40] and scaleErrorPct [1, -2, 0] take
    // Hakoniwa's output back to the specific force (2, 1, -9.81); unset, the value only turns.
    ArdupilotSensorModelV2Config acc_cal{};
    close(ap_correct_accel_sensor_to_body(acc_cal, {2.216133, 0.832900, -9.417734}), {2.216133, 0.832900, -9.417734},
        "missing INS_ACCOFFS/INS_ACCSCAL changed the accelerometer");
    acc_cal.acc_offset_mps2 = {0.196133, -0.147100, 0.392266};
    acc_cal.acc_scale = {0.990099, 1.020408, 1.0};
    close(ap_correct_accel_sensor_to_body(acc_cal, {2.216133, 0.832900, -9.417734}), {2.0, 1.0, -9.81},
        "T-ACC-U1-040: INS_ACCOFFS/INS_ACCSCAL did not take the output back to the specific force");
    const auto acc_loaded = ArdupilotControllerConfigLoader{}.load_from_text(with(base,
        R"("INS_ACCOFFS_X":0.196133,"INS_ACCOFFS_Y":-0.1471,"INS_ACCOFFS_Z":0.392266,)"
        R"("INS_ACCSCAL_X":0.990099,"INS_ACCSCAL_Y":1.020408,"INS_ACCSCAL_Z":1)"));
    close(acc_loaded.sensor_model_v2.acc_offset_mps2, {0.196133, -0.1471, 0.392266}, "INS_ACCOFFS was not loaded");
    close(acc_loaded.sensor_model_v2.acc_scale, {0.990099, 1.020408, 1.0}, "INS_ACCSCAL was not loaded");
    ArdupilotSensorModelV2Config custom{};
    custom.board_orientation = 101;
    custom.custom_rotation_enabled = true;
    custom.custom_rotation_deg = {0.0, 0.0, 90.0};
    close(ap_rotate_sensor_to_body(custom, {0.0, -1.0, 0.0}), {1.0, 0.0, 0.0}, "Custom 1 failed");
    ArdupilotSensorModelV2Config mag{};
    mag.mag_offset_milligauss = {-30.0, 15.0, -8.0};
    mag.mag_diagonal = {0.8, 1.25, 2.0};
    close(ap_correct_mag_sensor_to_body(mag, {0.3425, -0.015, 0.22450635}),
        {0.25, 0.0, 0.4330127}, "AP_Compass calibration failed");
    const auto loaded = ArdupilotControllerConfigLoader{}.load_from_text(with(base,
        R"("AHRS_ORIENTATION":2,"INS_POS1_X":0.1,"GPS1_POS_Z":-0.2)"));
    require(loaded.sensor_model_v2.board_orientation == 2, "orientation was not loaded");
    require(loaded.sensor_model_v2.imu_position_body_m[0] == 0.1, "IMU position was not loaded");
    require(loaded.sensor_model_v2.gps_position_body_m[2] == -0.2, "GPS position was not loaded");
    // Handoff: the exact set Drone PRO's generate_ardupilot_sensor_mount_config.py writes for a sample mount
    // (IMU 10/0/30 deg at 0.1/-0.05/0.02 m, GPS at -0.1/0/-0.2 m, COM 0.01/0/0.03 m, hard and soft iron).
    const auto generated = ArdupilotControllerConfigLoader{}.load_from_text(with(base,
        R"("AHRS_ORIENTATION": 101, "CUST_ROT_ENABLE": 1, "CUST_ROT1_ROLL": 10.0, "CUST_ROT1_PITCH": 0.0,)"
        R"("CUST_ROT1_YAW": 30.0, "INS_POS1_X": 0.09000000000000001, "INS_POS1_Y": -0.05,)"
        R"("INS_POS1_Z": -0.009999999999999998, "GPS1_POS_X": -0.11, "GPS1_POS_Y": 0.0, "GPS1_POS_Z": -0.23,)"
        R"("COMPASS_OFS_X": -30.0, "COMPASS_OFS_Y": 15.0, "COMPASS_OFS_Z": -8.0, "COMPASS_DIA_X": 0.8,)"
        R"("COMPASS_DIA_Y": 1.25, "COMPASS_DIA_Z": 2.0, "COMPASS_ODI_X": 0.0, "COMPASS_ODI_Y": 0.0,)"
        R"("COMPASS_ODI_Z": 0.0)"));
    const auto& v2 = generated.sensor_model_v2;
    require(v2.board_orientation == 101 && v2.custom_rotation_enabled, "generated orientation was not loaded");
    close(v2.custom_rotation_deg, {10.0, 0.0, 30.0}, "generated custom rotation was not loaded");
    close(v2.imu_position_body_m, {0.09, -0.05, -0.01}, "generated IMU position was not loaded");
    close(v2.gps_position_body_m, {-0.11, 0.0, -0.23}, "generated GPS position was not loaded");
    close(v2.mag_offset_milligauss, {-30.0, 15.0, -8.0}, "generated compass offsets were not loaded");
    close(v2.mag_diagonal, {0.8, 1.25, 2.0}, "generated compass diagonal was not loaded");
    close(v2.mag_offdiagonal, {0.0, 0.0, 0.0}, "generated compass off-diagonal was not loaded");
    // [SEN-BAR-320] The barometer expectations are loaded within ArduPilot's @Range.
    const auto baro = ArdupilotControllerConfigLoader{}.load_from_text(with(base,
        R"("EK3_HGT_DELAY": 50, "EK3_ALT_M_NSE": 0.5)"));
    require(baro.ekf.hgt_delay_ms == 50.0 && baro.ekf.baro_alt_noise_m == 0.5, "baro expectations were not loaded");
    for (const char* invalid : {R"("EK3_HGT_DELAY": 251)", R"("EK3_HGT_DELAY": 50.5)", R"("EK3_ALT_M_NSE": 0.05)"}) {
        bool refused = false;
        try { ArdupilotControllerConfigLoader{}.load_from_text(with(base, invalid)); }
        catch (const std::invalid_argument&) { refused = true; }
        require(refused, "an out-of-range barometer expectation was accepted");
    }
    // [SEN-BAR-326] How ArduCopter reads the barometer comes from runtime; each key is required and checked.
    const auto reading = ArdupilotControllerConfigLoader{}.load_from_text(base);
    require(reading.ekf.baro_update_hz == 10.0 && reading.ekf.baro_calibration_warmup_updates == 10
        && reading.ekf.baro_calibration_samples == 5, "the barometer reading was not loaded");
    for (const char* key : {"baro_update_hz", "baro_calibration_warmup_updates", "baro_calibration_samples"}) {
        const auto at = base.find(std::string("\"") + key + "\"");
        require(at != std::string::npos, "the fixture lacks a barometer reading key");
        const auto renamed = base.substr(0, at + 1) + "x_" + base.substr(at + 1);
        bool refused = false;
        try { ArdupilotControllerConfigLoader{}.load_from_text(renamed); }
        catch (const std::exception&) { refused = true; }
        require(refused, "a missing barometer reading key was accepted");
    }
    for (const char* invalid : {R"("baro_update_hz": 0)", R"("baro_calibration_warmup_updates": 1.5)",
             R"("baro_calibration_samples": 0)"}) {
        const std::string key = std::string(invalid).substr(0, std::string(invalid).find(':'));
        const auto at = base.find(key);
        const auto end = base.find_first_of(",}\n", at);
        bool refused = false;
        try { ArdupilotControllerConfigLoader{}.load_from_text(base.substr(0, at) + invalid + base.substr(end)); }
        catch (const std::invalid_argument&) { refused = true; }
        require(refused, "an invalid barometer reading was accepted");
    }
    // [SEN-GPS-385] The receiver's own lag (GPS1_DELAY_MS 0) comes from runtime, required, 0..250 ms.
    require(reading.ekf.gps_receiver_default_lag_ms == 220.0, "the receiver lag was not loaded");
    {
        const auto at = base.find("\"gps_receiver_default_lag_ms\"");
        require(at != std::string::npos, "the fixture lacks the receiver lag");
        bool refused = false;
        try { ArdupilotControllerConfigLoader{}.load_from_text(base.substr(0, at + 1) + "x_" + base.substr(at + 1)); }
        catch (const std::exception&) { refused = true; }
        require(refused, "a missing receiver lag was accepted");
        const auto end = base.find_first_of(",}\n", at);
        refused = false;
        try { ArdupilotControllerConfigLoader{}.load_from_text(base.substr(0, at) + R"("gps_receiver_default_lag_ms": 251)" + base.substr(end)); }
        catch (const std::invalid_argument&) { refused = true; }
        require(refused, "an out-of-range receiver lag was accepted");
    }
    // [SEN-GPS-380] T-GPS-U1-380: the fix as AP_GPS reads ArduPilot SITL's UBX-NAV-PVT (toward zero, receiver units).
    EkfHilGpsInput exact{};
    exact.lat_deg = 35.68123456789;
    exact.lon_deg = -139.76712345678;
    exact.alt_m = 40.12345;
    exact.vn_mps = 1.23456;
    exact.ve_mps = -0.98765;
    exact.vd_mps = -0.0005;
    exact.eph_m = 0.5004;
    exact.epv_m = 0.75049;
    exact.sacc_mps = 0.2509;
    const auto fix = ap_ublox_pvt_fix(exact);
    const auto near = [](double actual, double expected) { return std::abs(actual - expected) < 1.0e-6; };
    require(near(fix.lat_deg, 35.6812345) && near(fix.lon_deg, -139.7671234), "lat/lon are not 1e-7 deg toward zero");
    require(near(fix.alt_m, 40.12), "the altitude is not h_msl (mm) / 10 cm");
    require(near(fix.vn_mps, 1.234) && near(fix.ve_mps, -0.987) && fix.vd_mps == 0.0, "the velocity is not mm/s toward zero");
    require(near(fix.eph_m, 0.5) && near(fix.epv_m, 0.75), "the position accuracy is not mm");
    require(near(fix.sacc_mps, 0.25), "the speed accuracy is not mm/s (SITL's tenth must not be reproduced)");
    const auto defaults = ArdupilotControllerConfigLoader{}.load_from_text(base);
    require(defaults.ekf.hgt_delay_ms == 60.0 && defaults.ekf.baro_alt_noise_m == 0.0, "baro defaults changed");
    close(ap_rotate_sensor_to_body(defaults.sensor_model_v2, {1, 2, 3}), {1, 2, 3}, "defaults changed value");
    bool rejected = false;
    try { ArdupilotControllerConfigLoader{}.load_from_text(with(base, R"("AHRS_ORIENTATION":99)")); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "invalid AHRS_ORIENTATION was accepted");
    rejected = false;
    try { ArdupilotControllerConfigLoader{}.load_from_text(with(base, R"("AHRS_ORIENTATION":41)")); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "unlisted AHRS_ORIENTATION was accepted");
    rejected = false;
    try { ArdupilotControllerConfigLoader{}.load_from_text(with(base, R"("AHRS_ORIENTATION":101)")); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Custom 1 without enable was accepted");
    std::cout << "ap_sensor_model_v2_smoke: PASS\n";
    return EXIT_SUCCESS;
}
