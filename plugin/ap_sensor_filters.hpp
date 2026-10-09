// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// ArduPilot filters the IMU before its controllers see it (AP_InertialSensor,
// INS_GYRO_FILTER and INS_ACCEL_FILTER). Drone PRO hands the adapter the
// unfiltered rate and acceleration, so the plugin applies the same filter
// here: the second-order Butterworth low-pass of libraries/Filter/
// LowPassFilter2p.cpp (DigitalBiquadFilter), sampled at the stage's rate.

#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"

#include <algorithm>
#include <cmath>

namespace hakoniwa::drone::control_adapter::plugin_filters {

class Biquad2p {
public:
    // As DigitalBiquadFilter::compute_params: the cutoff stays under 0.4 of the
    // sample frequency; a cutoff of 0 passes the sample through.
    void configure(double sample_hz, double cutoff_hz)
    {
        constexpr double kPi = 3.14159265358979323846;
        cutoff_hz_ = std::min(cutoff_hz, sample_hz * 0.4);
        active_ = cutoff_hz_ > 0.0 && sample_hz > 0.0;
        initialised_ = false;
        if (!active_) {
            return;
        }
        const double ohm = std::tan(kPi / (sample_hz / cutoff_hz_));
        const double c = 1.0 + 2.0 * std::cos(kPi / 4.0) * ohm + ohm * ohm;
        b0_ = ohm * ohm / c;
        b1_ = 2.0 * b0_;
        b2_ = b0_;
        a1_ = 2.0 * (ohm * ohm - 1.0) / c;
        a2_ = (1.0 - 2.0 * std::cos(kPi / 4.0) * ohm + ohm * ohm) / c;
    }

    double apply(double sample)
    {
        if (!active_) {
            return sample;
        }
        if (!initialised_) {
            // DigitalBiquadFilter::reset(value): start at the steady state of the sample.
            delay1_ = delay2_ = sample * (1.0 / (1.0 + a1_ + a2_));
            initialised_ = true;
        }
        const double delay0 = sample - delay1_ * a1_ - delay2_ * a2_;
        const double output = delay0 * b0_ + delay1_ * b1_ + delay2_ * b2_;
        delay2_ = delay1_;
        delay1_ = delay0;
        return output;
    }

    void reset() { initialised_ = false; }
    double cutoff_hz() const { return active_ ? cutoff_hz_ : 0.0; }

private:
    bool active_{false};
    bool initialised_{false};
    double cutoff_hz_{0.0};
    double b0_{1.0}, b1_{0.0}, b2_{0.0}, a1_{0.0}, a2_{0.0};
    double delay1_{0.0}, delay2_{0.0};
};

// Rate control on the gyro filtered at INS_GYRO_FILTER.
class FilteredRateControl final : public IRateControlBackend {
public:
    FilteredRateControl(const ArdupilotControllerConfig& config)
        : inner_(config.rate_control)
    {
        for (auto* filter : {&p_, &q_, &r_}) {
            filter->configure(config.runtime.rate_hz, config.sensor_filter.gyro_cutoff_hz);
        }
    }
    void reset() override
    {
        inner_.reset();
        p_.reset();
        q_.reset();
        r_.reset();
    }
    BodyTorqueCommand run(const RateControlInput& input) override
    {
        RateControlInput filtered = input;
        filtered.rate.p = p_.apply(input.rate.p);
        filtered.rate.q = q_.apply(input.rate.q);
        filtered.rate.r = r_.apply(input.rate.r);
        return inner_.run(filtered);
    }

private:
    ApRateControlBackend inner_;
    Biquad2p p_, q_, r_;
};

// Altitude control on the vertical acceleration filtered at INS_ACCEL_FILTER.
class FilteredAltitudeControl final : public IAltitudeControlBackend {
public:
    FilteredAltitudeControl(const ArdupilotControllerConfig& config)
        : inner_(config.altitude_control)
    {
        az_.configure(config.runtime.altitude_hz, config.sensor_filter.accel_cutoff_hz);
    }
    void reset() override
    {
        inner_.reset();
        az_.reset();
    }
    NormalizedVerticalThrustCommand run(const AltitudeControlInput& input, double dt_sec) override
    {
        AltitudeControlInput filtered = input;
        filtered.acceleration.az = az_.apply(input.acceleration.az);
        return inner_.run(filtered, dt_sec);
    }

private:
    ApAltitudeControlBackend inner_;
    Biquad2p az_;
};

// 3D position control on the acceleration filtered at INS_ACCEL_FILTER
// (only its vertical component reaches a controller, as in AC_PosControl).
class FilteredPositionControl3D final : public IPositionControl3DBackend {
public:
    FilteredPositionControl3D(const ArdupilotControllerConfig& config)
        : inner_(config.altitude_control, config.horizontal_control)
    {
        for (auto* filter : {&ax_, &ay_, &az_}) {
            filter->configure(config.runtime.altitude_hz, config.sensor_filter.accel_cutoff_hz);
        }
    }
    void reset() override
    {
        inner_.reset();
        ax_.reset();
        ay_.reset();
        az_.reset();
    }
    PositionControl3DOutput run_position(const PositionControl3DPositionInput& input, double dt_sec) override
    {
        PositionControl3DPositionInput filtered = input;
        filter(filtered.state);
        return inner_.run_position(filtered, dt_sec);
    }
    PositionControl3DOutput run_velocity(const PositionControl3DVelocityInput& input, double dt_sec) override
    {
        PositionControl3DVelocityInput filtered = input;
        filter(filtered.state);
        return inner_.run_velocity(filtered, dt_sec);
    }

private:
    void filter(PositionControl3DState& state)
    {
        state.acceleration.x = ax_.apply(state.acceleration.x);
        state.acceleration.y = ay_.apply(state.acceleration.y);
        state.acceleration.z = az_.apply(state.acceleration.z);
    }
    ApPositionControl3DBackend inner_;
    Biquad2p ax_, ay_, az_;
};

}  // namespace hakoniwa::drone::control_adapter::plugin_filters
