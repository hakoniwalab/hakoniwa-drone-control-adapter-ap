// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P_2D.h>
#include <AC_PID/AC_PID_2D.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace hakoniwa::drone::control_adapter { namespace { constexpr double gravity=9.80665;
void limit(double& x,double& y,double m){double l=std::hypot(x,y);if(m>0&&l>m){x*=m/l;y*=m/l;}}
void validate(const ApHorizontalPositionControlBackendConfig& c){const double v[]{c.position_p,c.velocity_p,c.velocity_i,c.velocity_d,c.velocity_feed_forward,c.velocity_imax_mps2,c.velocity_error_filter_hz,c.velocity_derivative_filter_hz,c.speed_max_mps,c.acceleration_max_mps2,c.jerk_max_mps3,c.angle_max_rad};for(double x:v)if(!std::isfinite(x)||x<0)throw std::invalid_argument("invalid ArduPilot horizontal configuration");}
}
class ApHorizontalPositionControlBackend::Impl {public:explicit Impl(const ApHorizontalPositionControlBackendConfig& c):position(c.position_p),velocity(c.velocity_p,c.velocity_i,c.velocity_d,c.velocity_feed_forward,c.velocity_imax_mps2,c.velocity_error_filter_hz,c.velocity_derivative_filter_hz){position.set_kP(c.position_p);position.set_limits(c.speed_max_mps,c.acceleration_max_mps2,c.jerk_max_mps3);velocity.set_kP(c.velocity_p);velocity.set_kI(c.velocity_i);velocity.set_kD(c.velocity_d);velocity.set_ff(c.velocity_feed_forward);velocity.set_imax(c.velocity_imax_mps2);velocity.set_filt_E_hz(c.velocity_error_filter_hz);velocity.set_filt_D_hz(c.velocity_derivative_filter_hz);}AC_P_2D position;AC_PID_2D velocity;};
ApHorizontalPositionControlBackend::ApHorizontalPositionControlBackend(const ApHorizontalPositionControlBackendConfig& c){set_config(c);}ApHorizontalPositionControlBackend::~ApHorizontalPositionControlBackend()=default;
void ApHorizontalPositionControlBackend::reset(){acceleration_target_x_=acceleration_target_y_=0;impl_->velocity.reset_I();impl_->velocity.reset_filter();}
void ApHorizontalPositionControlBackend::set_config(const ApHorizontalPositionControlBackendConfig& c){validate(c);config_=c;impl_=std::make_unique<Impl>(c);reset();}
HorizontalTiltTarget ApHorizontalPositionControlBackend::run(const HorizontalPositionControlInput& in,double dt){if(!std::isfinite(dt)||dt<=0)throw std::invalid_argument("horizontal dt must be positive");double vx=in.target_velocity.vx,vy=in.target_velocity.vy;if(in.mode==HorizontalControlMode::Position){postype_t tx=in.target_position.x,ty=in.target_position.y;Vector2f correction=impl_->position.update_all(tx,ty,{float(in.position.x),float(in.position.y)});vx+=correction.x;vy+=correction.y;}limit(vx,vy,config_.speed_max_mps);Vector2f a=impl_->velocity.update_all({float(vx),float(vy)},{float(in.velocity.vx),float(in.velocity.vy)},dt,{});double ax=a.x,ay=a.y;limit(ax,ay,config_.acceleration_max_mps2);double dx=ax-acceleration_target_x_,dy=ay-acceleration_target_y_;limit(dx,dy,config_.jerk_max_mps3*dt);acceleration_target_x_+=dx;acceleration_target_y_+=dy;limit(acceleration_target_x_,acceleration_target_y_,gravity*std::tan(config_.angle_max_rad));double cy=std::cos(in.yaw_rad),sy=std::sin(in.yaw_rad),f=cy*acceleration_target_x_+sy*acceleration_target_y_,r=-sy*acceleration_target_x_+cy*acceleration_target_y_;double pitch=std::atan2(-f,gravity),roll=std::atan2(r*std::cos(pitch),gravity);return{std::clamp(roll,-config_.angle_max_rad,config_.angle_max_rad),std::clamp(pitch,-config_.angle_max_rad,config_.angle_max_rad)};}
}
