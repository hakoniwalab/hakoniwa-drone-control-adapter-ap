// SPDX-License-Identifier: GPL-3.0-or-later
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
}
ArdupilotControllerConfig ArdupilotControllerConfigLoader::load_from_file(const std::string& path) const {
    std::ifstream f(path); if (!f) throw std::runtime_error("failed to open config file: "+path);
    return load_from_text(std::string(std::istreambuf_iterator<char>(f), {}));
}
ArdupilotControllerConfig ArdupilotControllerConfigLoader::load_from_text(const std::string& s) const {
    ArdupilotControllerConfig c{};
    c.runtime.altitude_hz=number(s,"altitude_hz"); c.runtime.horizontal_hz=number(s,"horizontal_hz");
    c.runtime.attitude_hz=number(s,"attitude_hz"); c.runtime.rate_hz=number(s,"rate_hz");
    const double hover=number(s,"MOT_THST_HOVER"); c.altitude_control.hover_thrust=hover;
    c.attitude_control.angle_roll_p=number(s,"ATC_ANG_RLL_P"); c.attitude_control.angle_pitch_p=number(s,"ATC_ANG_PIT_P"); c.attitude_control.angle_yaw_p=number(s,"ATC_ANG_YAW_P");
    c.altitude_control.position_p=number(s,"PSC_POSZ_P"); c.altitude_control.velocity_p=number(s,"PSC_VELZ_P"); c.altitude_control.velocity_i=number(s,"PSC_VELZ_I"); c.altitude_control.velocity_d=number(s,"PSC_VELZ_D"); c.altitude_control.velocity_imax_mps2=number(s,"PSC_VELZ_IMAX"); c.altitude_control.velocity_filter_hz=number(s,"PSC_VELZ_FLTE"); c.altitude_control.velocity_derivative_filter_hz=number(s,"PSC_VELZ_FLTD"); c.altitude_control.velocity_feed_forward=number(s,"PSC_VELZ_FF");
    c.altitude_control.acceleration_p=number(s,"PSC_ACCZ_P"); c.altitude_control.acceleration_i=number(s,"PSC_ACCZ_I"); c.altitude_control.acceleration_d=number(s,"PSC_ACCZ_D"); c.altitude_control.acceleration_imax=number(s,"PSC_ACCZ_IMAX"); c.altitude_control.acceleration_target_filter_hz=number(s,"PSC_ACCZ_FLTT"); c.altitude_control.acceleration_error_filter_hz=number(s,"PSC_ACCZ_FLTE"); c.altitude_control.acceleration_derivative_filter_hz=number(s,"PSC_ACCZ_FLTD"); c.altitude_control.acceleration_feed_forward=number(s,"PSC_ACCZ_FF"); c.altitude_control.speed_up_mps=number(s,"WPNAV_SPEED_UP"); c.altitude_control.speed_down_mps=number(s,"WPNAV_SPEED_DN"); c.altitude_control.acceleration_max_mps2=number(s,"WPNAV_ACCEL_Z"); c.altitude_control.jerk_max_mps3=number(s,"PSC_JERK_Z");
    c.horizontal_control.position_p=number(s,"PSC_POSXY_P"); c.horizontal_control.velocity_p=number(s,"PSC_VELXY_P"); c.horizontal_control.velocity_i=number(s,"PSC_VELXY_I"); c.horizontal_control.velocity_d=number(s,"PSC_VELXY_D"); c.horizontal_control.velocity_feed_forward=number(s,"PSC_VELXY_FF"); c.horizontal_control.velocity_imax_mps2=number(s,"PSC_VELXY_IMAX"); c.horizontal_control.velocity_error_filter_hz=number(s,"PSC_VELXY_FLTE"); c.horizontal_control.velocity_derivative_filter_hz=number(s,"PSC_VELXY_FLTD"); c.horizontal_control.speed_max_mps=number(s,"PSC_VELXY_MAX"); c.horizontal_control.acceleration_max_mps2=number(s,"PSC_ACCXY_MAX"); c.horizontal_control.jerk_max_mps3=number(s,"PSC_JERK_XY"); c.horizontal_control.angle_max_rad=number(s,"ANGLE_MAX_RAD");
    auto axis=[&](ApRateControlAxisConfig& a,const char* p,const char* i,const char* d,const char* ff,const char* imax,const char* fltt,const char* flte,const char* fltd,const char* smax,const char* tau){a.p=number(s,p);a.i=number(s,i);a.d=number(s,d);a.feed_forward=number(s,ff);a.integrator_limit=number(s,imax);a.target_filter_hz=number(s,fltt);a.error_filter_hz=number(s,flte);a.derivative_filter_hz=number(s,fltd);a.slew_rate_max=number(s,smax);a.slew_rate_tau=number(s,tau);};
    axis(c.rate_control.roll,"ATC_RAT_RLL_P","ATC_RAT_RLL_I","ATC_RAT_RLL_D","ATC_RAT_RLL_FF","ATC_RAT_RLL_IMAX","ATC_RAT_RLL_FLTT","ATC_RAT_RLL_FLTE","ATC_RAT_RLL_FLTD","ATC_RAT_RLL_SMAX","ATC_RAT_RLL_SRT");
    axis(c.rate_control.pitch,"ATC_RAT_PIT_P","ATC_RAT_PIT_I","ATC_RAT_PIT_D","ATC_RAT_PIT_FF","ATC_RAT_PIT_IMAX","ATC_RAT_PIT_FLTT","ATC_RAT_PIT_FLTE","ATC_RAT_PIT_FLTD","ATC_RAT_PIT_SMAX","ATC_RAT_PIT_SRT");
    axis(c.rate_control.yaw,"ATC_RAT_YAW_P","ATC_RAT_YAW_I","ATC_RAT_YAW_D","ATC_RAT_YAW_FF","ATC_RAT_YAW_IMAX","ATC_RAT_YAW_FLTT","ATC_RAT_YAW_FLTE","ATC_RAT_YAW_FLTD","ATC_RAT_YAW_SMAX","ATC_RAT_YAW_SRT");
    return c;
}
}  // namespace hakoniwa::drone::control_adapter
