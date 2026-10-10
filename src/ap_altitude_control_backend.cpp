// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include <AC_PID/AC_P_1D.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace hakoniwa::drone::control_adapter { namespace {
void validate(const ApAltitudeControlBackendConfig& c) { const double v[]{c.position_p,c.velocity_p,c.velocity_i,c.velocity_d,c.velocity_imax_mps2,c.velocity_filter_hz,c.velocity_derivative_filter_hz,c.velocity_feed_forward,c.acceleration_p,c.acceleration_i,c.acceleration_d,c.acceleration_imax,c.acceleration_target_filter_hz,c.acceleration_error_filter_hz,c.acceleration_derivative_filter_hz,c.acceleration_feed_forward,c.hover_thrust,c.speed_up_mps,c.speed_down_mps,c.acceleration_max_mps2,c.jerk_max_mps3,c.thrust_min,c.thrust_max}; for(double x:v)if(!std::isfinite(x)||x<0)throw std::invalid_argument("invalid ArduPilot altitude configuration"); if(c.hover_thrust<=0||c.thrust_min>c.thrust_max)throw std::invalid_argument("invalid ArduPilot altitude thrust range"); }
AC_PID::Defaults defs(double p,double i,double d,double ff,double imax,double tf,double ef,double df){return {float(p),float(i),float(d),float(ff),float(imax),float(tf),float(ef),float(df),0,1,0};}
void configure(AC_PID& p,double kp,double ki,double kd,double ff,double imax,double tf,double ef,double df){p.set_kP(kp);p.set_kI(ki);p.set_kD(kd);p.set_ff(ff);p.set_imax(imax);p.set_filt_T_hz(tf);p.set_filt_E_hz(ef);p.set_filt_D_hz(df);}
}
class ApAltitudeControlBackend::Impl { public: explicit Impl(const ApAltitudeControlBackendConfig& c):position(c.position_p),velocity(defs(c.velocity_p,c.velocity_i,c.velocity_d,c.velocity_feed_forward,c.velocity_imax_mps2,c.velocity_filter_hz,c.velocity_filter_hz,c.velocity_derivative_filter_hz)),acceleration(defs(c.acceleration_p,c.acceleration_i,c.acceleration_d,c.acceleration_feed_forward,c.acceleration_imax*1000,c.acceleration_target_filter_hz,c.acceleration_error_filter_hz,c.acceleration_derivative_filter_hz)){position.set_kP(c.position_p);position.set_limits(-c.speed_down_mps,c.speed_up_mps,c.acceleration_max_mps2,0.0f);configure(velocity,c.velocity_p,c.velocity_i,c.velocity_d,c.velocity_feed_forward,c.velocity_imax_mps2,c.velocity_filter_hz,c.velocity_filter_hz,c.velocity_derivative_filter_hz);configure(acceleration,c.acceleration_p,c.acceleration_i,c.acceleration_d,c.acceleration_feed_forward,c.acceleration_imax*1000,c.acceleration_target_filter_hz,c.acceleration_error_filter_hz,c.acceleration_derivative_filter_hz);acceleration.set_imax(std::max(c.acceleration_imax,c.hover_thrust)*1000);} AC_P_1D position; AC_PID velocity,acceleration; };
ApAltitudeControlBackend::ApAltitudeControlBackend(const ApAltitudeControlBackendConfig& c){set_config(c);} ApAltitudeControlBackend::~ApAltitudeControlBackend()=default;
void ApAltitudeControlBackend::reset(){shaping_started_=false;acceleration_target_mps2_=0;throttle_filter_started_=false;impl_->velocity.reset_I();impl_->velocity.reset_filter();impl_->acceleration.reset_I();impl_->acceleration.reset_filter();}
NormalizedVerticalThrustCommand ApAltitudeControlBackend::run(const AltitudeControlInput& in,double dt){if(!std::isfinite(dt)||dt<=0)throw std::invalid_argument("altitude dt must be positive"); double vt=in.target_velocity.vz; double shaped_accel=0.0;
if(in.mode!=AltitudeControlMode::Position)shaping_started_=false;
if(in.mode==AltitudeControlMode::Position){double target_alt=in.target_altitude;
// AC_PosControl::input_pos_xyz (z): advance the desired altitude, then shape it toward the target.
if(config_.input_shaping){if(!shaping_started_){shaped_pos_up_=in.position.z;shaped_vel_up_=in.velocity.vz;shaped_accel_up_=0;shaping_started_=true;}
postype_t pos=shaped_pos_up_;float vel=float(shaped_vel_up_);float acc=float(shaped_accel_up_);const float f_dt=float(dt);
update_pos_vel_accel(pos,vel,acc,f_dt,0.0f,0.0f,0.0f);
const float accel_max=float(config_.acceleration_max_mps2);
shape_pos_vel_accel(postype_t(target_alt),0.0f,0.0f,pos,vel,acc,-float(config_.speed_down_mps),float(config_.speed_up_mps),-constrain_float(accel_max,0.0f,7.5f),accel_max,float(config_.jerk_max_mps3),f_dt,false);
shaped_pos_up_=pos;shaped_vel_up_=vel;shaped_accel_up_=acc;target_alt=pos;vt+=vel;shaped_accel=acc;}
float target=float(target_alt);vt+=impl_->position.update_all(target,in.position.z);}vt=std::clamp(vt,-config_.speed_down_mps,config_.speed_up_mps);// As AC_PosControl::update_z_controller: the velocity PID output is the acceleration target as it is.
// The acceleration and jerk limits shape a target trajectory in ArduPilot (input shaping), never the
// feedback path; a slew limit here acts as an amplitude-dependent delay and makes large steps oscillate.
// AC_PosControl::update_z_controller hands the previous cycle's motor limits to the PIDs: the
// velocity PID (AC_PID_Basic, directional) stops integrating toward a limited side; the
// acceleration PID's integrator may only shrink while either side is limited.
const bool limit_lower=motor_limits_&&motor_limits_->lower, limit_upper=motor_limits_&&motor_limits_->upper;
const double velocity_error=vt-in.velocity.vz;
const bool velocity_limit=(limit_lower&&velocity_error<0)||(limit_upper&&velocity_error>0);
acceleration_target_mps2_=impl_->velocity.update_all(vt,in.velocity.vz,dt,velocity_limit)+impl_->velocity.get_ff()+acceleration_feedforward_mps2_+shaped_accel;double throttle=config_.hover_thrust+.001*(impl_->acceleration.update_all(acceleration_target_mps2_*100,in.acceleration.az*100,dt,limit_lower||limit_upper)+impl_->acceleration.get_ff());// AC_AttitudeControl_Multi::get_throttle_boosted: inverted_factor from the current tilt (fades the
// boost out between 60 and 90 deg), boost from the target thrust angle, its cosine held to 0.1..1.
const double cos_tilt=1-2*(in.attitude.x*in.attitude.x+in.attitude.y*in.attitude.y);
const double inverted_factor=std::clamp(10.0*cos_tilt,0.0,1.0);
const double cos_tilt_target=has_target_thrust_angle_?std::cos(target_thrust_angle_rad_):cos_tilt;
throttle*=inverted_factor/std::clamp(cos_tilt_target,0.1,1.0);
// AP_Motors::update_throttle_filter: the boosted throttle through the 2 Hz first-order low-pass
// (LowPassFilterFloat, alpha = dt / (dt + 1/(2 pi fc))), held to 0..1. The adapter starts the
// filter at its first throttle (Drone PRO may start the vehicle in the air), where ArduPilot
// starts it at 0 on the ground.
if(config_.throttle_filter_hz>0){if(!throttle_filter_started_){throttle_filtered_=throttle;throttle_filter_started_=true;}
const double rc=1.0/(2.0*3.14159265358979323846*config_.throttle_filter_hz);throttle_filtered_+=(throttle-throttle_filtered_)*(dt/(dt+rc));
throttle_filtered_=std::clamp(throttle_filtered_,0.0,1.0);throttle=throttle_filtered_;}
throttle=std::clamp(throttle,config_.thrust_min,config_.thrust_max);return{-throttle/config_.hover_thrust};}
void ApAltitudeControlBackend::set_config(const ApAltitudeControlBackendConfig& c){validate(c);config_=c;impl_=std::make_unique<Impl>(c);reset();}
}
