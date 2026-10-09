// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/ap_rate_control_backend.hpp"

#include <AC_PID/AC_PID.h>

#include <cmath>
#include <stdexcept>

namespace hakoniwa::drone::control_adapter {

namespace {

AC_PID::Defaults to_ap_defaults(const ApRateControlAxisConfig& config)
{
    return AC_PID::Defaults{
        static_cast<float>(config.p),
        static_cast<float>(config.i),
        static_cast<float>(config.d),
        static_cast<float>(config.feed_forward),
        static_cast<float>(config.integrator_limit),
        static_cast<float>(config.target_filter_hz),
        static_cast<float>(config.error_filter_hz),
        static_cast<float>(config.derivative_filter_hz),
        static_cast<float>(config.slew_rate_max),
        static_cast<float>(config.slew_rate_tau),
        static_cast<float>(config.derivative_feed_forward)
    };
}

bool is_limited(const AxisSaturationFlags& saturation)
{
    // ArduPilot 4.6.3 exposes one saturation flag per rate axis. Its
    // integrator may shrink while limited, but it may not grow further.
    return saturation.positive || saturation.negative;
}

void apply_config(AC_PID& controller, const ApRateControlAxisConfig& config)
{
    // The adapter configuration is authoritative. The small runtime shim does
    // not provide ArduPilot's EEPROM-backed AP_Param store, so every value used
    // by the controller is applied explicitly here.
    controller.set_kP(static_cast<float>(config.p));
    controller.set_kI(static_cast<float>(config.i));
    controller.set_kD(static_cast<float>(config.d));
    controller.set_ff(static_cast<float>(config.feed_forward));
    controller.set_imax(static_cast<float>(config.integrator_limit));
    controller.set_filt_T_hz(static_cast<float>(config.target_filter_hz));
    controller.set_filt_E_hz(static_cast<float>(config.error_filter_hz));
    controller.set_filt_D_hz(static_cast<float>(config.derivative_filter_hz));
    controller.set_slew_limit(static_cast<float>(config.slew_rate_max));
    controller.set_kDff(static_cast<float>(config.derivative_feed_forward));
}

double run_axis(
    AC_PID& controller,
    double target,
    double measured,
    double dt_sec,
    const AxisSaturationFlags& saturation)
{
    const float feedback = controller.update_all(
        static_cast<float>(target),
        static_cast<float>(measured),
        static_cast<float>(dt_sec),
        is_limited(saturation));

    // AC_AttitudeControl_Multi sends update_all() and get_ff() through two
    // motor inputs which AP_Motors later sums. The adapter returns that same
    // combined command at its public backend boundary.
    return static_cast<double>(feedback + controller.get_ff());
}

ApRateControlAxisStatus status_of(const AC_PID& controller)
{
    return ApRateControlAxisStatus{
        controller.get_p(),
        controller.get_i(),
        controller.get_d(),
        controller.get_ff()
    };
}

}  // namespace

class ApRateControlBackend::Impl {
public:
    explicit Impl(const ApRateControlBackendConfig& config)
        : roll(to_ap_defaults(config.roll))
        , pitch(to_ap_defaults(config.pitch))
        , yaw(to_ap_defaults(config.yaw))
    {
        apply_config(roll, config.roll);
        apply_config(pitch, config.pitch);
        apply_config(yaw, config.yaw);
    }

    AC_PID roll;
    AC_PID pitch;
    AC_PID yaw;
};

ApRateControlBackend::ApRateControlBackend(const ApRateControlBackendConfig& config)
{
    set_config(config);
}

ApRateControlBackend::~ApRateControlBackend() = default;

void ApRateControlBackend::reset()
{
    impl_->roll.reset_I();
    impl_->pitch.reset_I();
    impl_->yaw.reset_I();
    impl_->roll.reset_filter();
    impl_->pitch.reset_filter();
    impl_->yaw.reset_filter();
}

BodyTorqueCommand ApRateControlBackend::run(const RateControlInput& input)
{
    if (!std::isfinite(input.dt_sec) || input.dt_sec <= 0.0) {
        throw std::invalid_argument("ArduPilot rate control requires dt_sec > 0");
    }

    if (input.landed) {
        // AC_AttitudeControl resets its rate integrators while the vehicle is
        // landed. Keep the output proportional/feed-forward but never retain
        // ground error for the next airborne cycle.
        impl_->roll.reset_I();
        impl_->pitch.reset_I();
        impl_->yaw.reset_I();
    }

    const BodyTorqueCommand output{
        run_axis(
            impl_->roll,
            input.target.p,
            input.rate.p,
            input.dt_sec,
            input.saturation.roll),
        run_axis(
            impl_->pitch,
            input.target.q,
            input.rate.q,
            input.dt_sec,
            input.saturation.pitch),
        run_axis(
            impl_->yaw,
            input.target.r,
            input.rate.r,
            input.dt_sec,
            input.saturation.yaw)
    };
    if (input.landed) {
        impl_->roll.reset_I();
        impl_->pitch.reset_I();
        impl_->yaw.reset_I();
    }
    return output;
}

void ApRateControlBackend::set_config(const ApRateControlBackendConfig& config)
{
    config_ = config;
    impl_ = std::make_unique<Impl>(config);
    reset();
}

ApRateControlBackendStatus ApRateControlBackend::get_status() const
{
    return ApRateControlBackendStatus{
        status_of(impl_->roll),
        status_of(impl_->pitch),
        status_of(impl_->yaw)
    };
}

}  // namespace hakoniwa::drone::control_adapter
