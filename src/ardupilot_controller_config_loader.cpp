// SPDX-License-Identifier: GPL-3.0-or-later
#include <cmath>
#include <stdexcept>
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include <fstream>
#include <regex>
#include <stdexcept>
namespace hakoniwa::drone::control_adapter {
namespace {
double number(const std::string& s, const char* key) {
    std::smatch m; const std::regex r("\\\"" + std::string(key) + "\\\"\\s*:\\s*([-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?)");
    if (!std::regex_search(s,m,r)) throw std::runtime_error(std::string("missing required config key: ")+key);
    return std::stod(m[1].str());
}
double optional_number(const std::string& s, const char* key, double fallback) {
    std::smatch m; const std::regex r("\\\"" + std::string(key) + "\\\"\\s*:\\s*([-+]?(?:\\d+\\.?\\d*|\\d*\\.\\d+)(?:[eE][-+]?\\d+)?)");
    return std::regex_search(s,m,r) ? std::stod(m[1].str()) : fallback;
}
std::string optional_string(const std::string& s, const char* key) {
    std::smatch m; const std::regex r("\\\"" + std::string(key) + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    return std::regex_search(s,m,r) ? m[1].str() : std::string{};
}
}
ArdupilotControllerConfig ArdupilotControllerConfigLoader::load_from_file(const std::string& path) const {
    std::ifstream f(path); if (!f) throw std::runtime_error("failed to open config file: "+path);
    return load_from_text(std::string(std::istreambuf_iterator<char>(f), {}));
}
ArdupilotControllerConfig ArdupilotControllerConfigLoader::load_from_text(const std::string& s) const {
    ArdupilotControllerConfig c{};
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    auto& sensor = c.sensor_model_v2;
    const double orientation = optional_number(s, "AHRS_ORIENTATION", 0.0);
    if (!std::isfinite(orientation) || orientation != std::trunc(orientation)) {
        throw std::invalid_argument("AHRS_ORIENTATION must be an integer");
    }
    sensor.board_orientation = static_cast<int>(orientation);
    const bool standard_orientation = (sensor.board_orientation >= 0 && sensor.board_orientation <= 37)
        || sensor.board_orientation == 39 || sensor.board_orientation == 40
        || sensor.board_orientation == 42 || sensor.board_orientation == 43;
    if (!(standard_orientation || sensor.board_orientation == 101)) {
        throw std::invalid_argument("AHRS_ORIENTATION must be a supported Rotation or 101 (Custom 1)");
    }
    const double custom_rotation_enable = optional_number(s, "CUST_ROT_ENABLE", 0.0);
    if (custom_rotation_enable != 0.0 && custom_rotation_enable != 1.0) {
        throw std::invalid_argument("CUST_ROT_ENABLE must be 0 or 1");
    }
    sensor.custom_rotation_enabled = custom_rotation_enable == 1.0;
    sensor.custom_rotation_deg = {
        optional_number(s, "CUST_ROT1_ROLL", 0.0),
        optional_number(s, "CUST_ROT1_PITCH", 0.0),
        optional_number(s, "CUST_ROT1_YAW", 0.0)};
    if (sensor.board_orientation == 101 && !sensor.custom_rotation_enabled) {
        throw std::invalid_argument("AHRS_ORIENTATION 101 requires CUST_ROT_ENABLE=1");
    }
    for (const double angle : sensor.custom_rotation_deg) {
        if (!std::isfinite(angle) || angle < -360.0 || angle > 360.0) {
            throw std::invalid_argument("CUST_ROT1_ROLL/PITCH/YAW must be finite and within -360..360 degrees");
        }
    }
    sensor.imu_position_body_m = {
        optional_number(s, "INS_POS1_X", 0.0), optional_number(s, "INS_POS1_Y", 0.0),
        optional_number(s, "INS_POS1_Z", 0.0)};
    sensor.gps_position_body_m = {
        optional_number(s, "GPS1_POS_X", 0.0), optional_number(s, "GPS1_POS_Y", 0.0),
        optional_number(s, "GPS1_POS_Z", 0.0)};
    sensor.mag_offset_milligauss = {
        optional_number(s, "COMPASS_OFS_X", 0.0), optional_number(s, "COMPASS_OFS_Y", 0.0),
        optional_number(s, "COMPASS_OFS_Z", 0.0)};
    sensor.mag_diagonal = {
        optional_number(s, "COMPASS_DIA_X", 1.0), optional_number(s, "COMPASS_DIA_Y", 1.0),
        optional_number(s, "COMPASS_DIA_Z", 1.0)};
    sensor.mag_offdiagonal = {
        optional_number(s, "COMPASS_ODI_X", 0.0), optional_number(s, "COMPASS_ODI_Y", 0.0),
        optional_number(s, "COMPASS_ODI_Z", 0.0)};
    for (const auto& values : {sensor.imu_position_body_m, sensor.gps_position_body_m,
             sensor.mag_offset_milligauss, sensor.mag_diagonal, sensor.mag_offdiagonal}) {
        for (const double value : values) {
            if (!std::isfinite(value)) throw std::invalid_argument("sensor model parameters must be finite");
        }
    }
#endif
    c.runtime.altitude_hz=number(s,"altitude_hz"); c.runtime.horizontal_hz=number(s,"horizontal_hz");
    c.runtime.attitude_hz=number(s,"attitude_hz"); c.runtime.rate_hz=number(s,"rate_hz");
    c.sensor_filter.gyro_cutoff_hz=optional_number(s,"INS_GYRO_FILTER",0.0); c.sensor_filter.accel_cutoff_hz=optional_number(s,"INS_ACCEL_FILTER",0.0);
    c.position_shaping.enabled=optional_number(s,"position_input_shaping",1.0)!=0.0;
    c.altitude_control.input_shaping=c.position_shaping.enabled;  // the altitude stage on its own shapes its target too
    // In the adapter's units (m/s^2), as WPNAV_ACCEL_Z for the altitude stage; the exporter writes cm/s^2.
    c.position_shaping.acceleration_xy_mps2=optional_number(s,"WPNAV_ACCEL",0.0);  // 0: from the lean limit
    c.position_shaping.acceleration_z_mps2=optional_number(s,"WPNAV_ACCEL_Z",2.5);
    c.position_shaping.jerk_xy_mps3=optional_number(s,"PSC_JERK_XY",5.0); c.position_shaping.jerk_z_mps3=optional_number(s,"PSC_JERK_Z",5.0);
    if(!(c.position_shaping.acceleration_xy_mps2>=0)||!(c.position_shaping.acceleration_z_mps2>0)||!(c.position_shaping.jerk_xy_mps3>0)||!(c.position_shaping.jerk_z_mps3>0))throw std::invalid_argument("WPNAV_ACCEL must be >= 0; WPNAV_ACCEL_Z, PSC_JERK_XY and PSC_JERK_Z must be positive");
    c.flight_phase.takeoff_profile=optional_number(s,"takeoff_profile",1.0)!=0.0;
    c.flight_phase.landing_profile=optional_number(s,"landing_profile",1.0)!=0.0;
    c.flight_phase.arming_delay_sec=optional_number(s,"ARMING_DELAY_SEC",2.0);
    c.flight_phase.motor_spool_time_sec=optional_number(s,"MOT_SPOOL_TIME",0.5);
    c.flight_phase.takeoff_slew_time_sec=optional_number(s,"TKOFF_SLEW_TIME",2.0);
    c.flight_phase.takeoff_slew_loop_rate_hz=optional_number(s,"takeoff_slew_loop_rate_hz",0.0);
    c.flight_phase.takeoff_throttle_max=optional_number(s,"TKOFF_THR_MAX",0.9);
    c.flight_phase.land_speed_mps=optional_number(s,"LAND_SPEED",0.5);
    c.flight_phase.land_speed_high_mps=optional_number(s,"LAND_SPEED_HIGH",0.0);
    c.flight_phase.land_alt_low_m=optional_number(s,"LAND_ALT_LOW",10.0);
    c.flight_phase.motor_spin_arm=optional_number(s,"MOT_SPIN_ARM",0.10);
    c.flight_phase.motor_spin_min=optional_number(s,"MOT_SPIN_MIN",0.15);
    c.flight_phase.motor_spin_max=optional_number(s,"MOT_SPIN_MAX",0.95);
    c.flight_phase.motor_thrust_expo=optional_number(s,"MOT_THST_EXPO",0.65);
    c.flight_phase.motor_model_a_coef=optional_number(s,"motor_model_a_coef",0.0);
    c.flight_phase.motor_model_b_coef=optional_number(s,"motor_model_b_coef",0.0);
    c.flight_phase.motor_nominal_voltage=optional_number(s,"motor_nominal_voltage",0.0);
    c.flight_phase.motor_hover_omega=optional_number(s,"motor_hover_omega",0.0);
    c.flight_phase.ground_motor_output=
        c.flight_phase.motor_model_b_coef>0.0 && c.flight_phase.motor_nominal_voltage>0.0 &&
        c.flight_phase.motor_hover_omega>0.0;
    c.control_allocation.ground_idle_thrust_max=optional_number(s,"spin_min_thrust_hover",0.0);
    c.ekf.gps_delay_ms=optional_number(s,"GPS1_DELAY_MS",0.0); c.ekf.hgt_delay_ms=optional_number(s,"EK3_HGT_DELAY",60.0);
    c.ekf.debug_csv=optional_string(s,"ekf_debug_csv");
    if(!std::isfinite(c.ekf.gps_delay_ms)||c.ekf.gps_delay_ms<0||c.ekf.gps_delay_ms>250)throw std::invalid_argument("GPS1_DELAY_MS must be 0..250 (NavEKF3 limit)");
    // NavEKF3 inside the plugin runs with its default parameters; its height delay cannot be changed.
    if(c.ekf.hgt_delay_ms!=60.0)throw std::invalid_argument("EK3_HGT_DELAY must be 60 (the plugin's NavEKF3 uses the default); set the barometer delay to 60 ms instead");
    const double hover=number(s,"MOT_THST_HOVER"); c.altitude_control.hover_thrust=hover;
    c.attitude_control.angle_roll_p=number(s,"ATC_ANG_RLL_P"); c.attitude_control.angle_pitch_p=number(s,"ATC_ANG_PIT_P"); c.attitude_control.angle_yaw_p=number(s,"ATC_ANG_YAW_P");
    // ATC_ACCEL_*_MAX (centidegrees/s^2, ArduPilot defaults AC_ATTITUDE_CONTROL_ACCEL_RP/Y_MAX_DEFAULT_CDSS): the
    // attitude-error-to-rate sqrt controller clamps the limit into 40..720 deg/s^2, so an absent key must give
    // ArduPilot's default and not 0 (0 would be clamped to 40 deg/s^2, a sluggish vehicle).
    const double cdss_to_rad=3.14159265358979323846/18000.0;
    c.attitude_control.accel_roll_max_rad_sec2=optional_number(s,"ATC_ACCEL_R_MAX",110000.0)*cdss_to_rad;
    c.attitude_control.accel_pitch_max_rad_sec2=optional_number(s,"ATC_ACCEL_P_MAX",110000.0)*cdss_to_rad;
    c.attitude_control.accel_yaw_max_rad_sec2=optional_number(s,"ATC_ACCEL_Y_MAX",27000.0)*cdss_to_rad;
    // AC_AttitudeControl input shaping, with ArduPilot's defaults when absent (ATC_RATE_FF_ENAB 1,
    // ATC_INPUT_TC 0.15 s, ATC_SLEW_YAW 6000 cdeg/s).
    c.attitude_control.rate_feedforward_enabled=optional_number(s,"ATC_RATE_FF_ENAB",1.0)!=0.0;
    c.attitude_control.input_time_constant_sec=optional_number(s,"ATC_INPUT_TC",0.15);
    c.attitude_control.slew_yaw_rad_sec=optional_number(s,"ATC_SLEW_YAW",6000.0)*cdss_to_rad;
    // AC_PosControl's throttle low-pass (POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ, a constant in ArduPilot).
    // An adapter-only key so a configuration can leave it out (0), as the interface contract test does.
    c.altitude_control.throttle_filter_hz=optional_number(s,"throttle_filter_hz",2.0);
    c.altitude_control.position_p=number(s,"PSC_POSZ_P"); c.altitude_control.velocity_p=number(s,"PSC_VELZ_P"); c.altitude_control.velocity_i=number(s,"PSC_VELZ_I"); c.altitude_control.velocity_d=number(s,"PSC_VELZ_D"); c.altitude_control.velocity_imax_mps2=number(s,"PSC_VELZ_IMAX"); c.altitude_control.velocity_filter_hz=number(s,"PSC_VELZ_FLTE"); c.altitude_control.velocity_derivative_filter_hz=number(s,"PSC_VELZ_FLTD"); c.altitude_control.velocity_feed_forward=number(s,"PSC_VELZ_FF");
    c.altitude_control.acceleration_p=number(s,"PSC_ACCZ_P"); c.altitude_control.acceleration_i=number(s,"PSC_ACCZ_I"); c.altitude_control.acceleration_d=number(s,"PSC_ACCZ_D"); c.altitude_control.acceleration_imax=number(s,"PSC_ACCZ_IMAX"); c.altitude_control.acceleration_target_filter_hz=number(s,"PSC_ACCZ_FLTT"); c.altitude_control.acceleration_error_filter_hz=number(s,"PSC_ACCZ_FLTE"); c.altitude_control.acceleration_derivative_filter_hz=number(s,"PSC_ACCZ_FLTD"); c.altitude_control.acceleration_feed_forward=number(s,"PSC_ACCZ_FF"); c.altitude_control.speed_up_mps=number(s,"WPNAV_SPEED_UP"); c.altitude_control.speed_down_mps=number(s,"WPNAV_SPEED_DN"); c.altitude_control.acceleration_max_mps2=number(s,"WPNAV_ACCEL_Z"); c.altitude_control.jerk_max_mps3=number(s,"PSC_JERK_Z");
    c.horizontal_control.position_p=number(s,"PSC_POSXY_P"); c.horizontal_control.velocity_p=number(s,"PSC_VELXY_P"); c.horizontal_control.velocity_i=number(s,"PSC_VELXY_I"); c.horizontal_control.velocity_d=number(s,"PSC_VELXY_D"); c.horizontal_control.velocity_feed_forward=number(s,"PSC_VELXY_FF"); c.horizontal_control.velocity_imax_mps2=number(s,"PSC_VELXY_IMAX"); c.horizontal_control.velocity_error_filter_hz=number(s,"PSC_VELXY_FLTE"); c.horizontal_control.velocity_derivative_filter_hz=number(s,"PSC_VELXY_FLTD"); c.horizontal_control.speed_max_mps=number(s,"PSC_VELXY_MAX"); c.horizontal_control.acceleration_max_mps2=number(s,"PSC_ACCXY_MAX"); c.horizontal_control.jerk_max_mps3=number(s,"PSC_JERK_XY"); c.horizontal_control.angle_max_rad=number(s,"ANGLE_MAX_RAD");
    auto axis=[&](ApRateControlAxisConfig& a,const char* p,const char* i,const char* d,const char* ff,const char* imax,const char* fltt,const char* flte,const char* fltd,const char* smax,const char* tau){a.p=number(s,p);a.i=number(s,i);a.d=number(s,d);a.feed_forward=number(s,ff);a.integrator_limit=number(s,imax);a.target_filter_hz=number(s,fltt);a.error_filter_hz=number(s,flte);a.derivative_filter_hz=number(s,fltd);a.slew_rate_max=number(s,smax);a.slew_rate_tau=number(s,tau);};
    axis(c.rate_control.roll,"ATC_RAT_RLL_P","ATC_RAT_RLL_I","ATC_RAT_RLL_D","ATC_RAT_RLL_FF","ATC_RAT_RLL_IMAX","ATC_RAT_RLL_FLTT","ATC_RAT_RLL_FLTE","ATC_RAT_RLL_FLTD","ATC_RAT_RLL_SMAX","ATC_RAT_RLL_SRT");
    axis(c.rate_control.pitch,"ATC_RAT_PIT_P","ATC_RAT_PIT_I","ATC_RAT_PIT_D","ATC_RAT_PIT_FF","ATC_RAT_PIT_IMAX","ATC_RAT_PIT_FLTT","ATC_RAT_PIT_FLTE","ATC_RAT_PIT_FLTD","ATC_RAT_PIT_SMAX","ATC_RAT_PIT_SRT");
    axis(c.rate_control.yaw,"ATC_RAT_YAW_P","ATC_RAT_YAW_I","ATC_RAT_YAW_D","ATC_RAT_YAW_FF","ATC_RAT_YAW_IMAX","ATC_RAT_YAW_FLTT","ATC_RAT_YAW_FLTE","ATC_RAT_YAW_FLTD","ATC_RAT_YAW_SMAX","ATC_RAT_YAW_SRT");
    c.rate_control.previous_rates_setpoint=optional_number(s,"previous_rates_setpoint",0.0)!=0.0;
    return c;
}
}  // namespace hakoniwa::drone::control_adapter
