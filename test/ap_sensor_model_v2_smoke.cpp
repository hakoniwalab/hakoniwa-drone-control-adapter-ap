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
    const auto defaults = ArdupilotControllerConfigLoader{}.load_from_text(base);
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
