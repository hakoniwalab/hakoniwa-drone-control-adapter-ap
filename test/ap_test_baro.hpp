// SPDX-License-Identifier: GPL-3.0-or-later
// Barometer input for the EKF3 smoke tests in both builds.
#pragma once

#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>

#include <cmath>
#include <cstdint>

namespace hakoniwa::drone::control_adapter::test {

// ISA troposphere pressure at height_m above sea level (Drone PRO's barometer output at rest, no noise).
inline double isa_pressure_pa(double height_m)
{
    return 101325.0 * std::pow(1.0 - 2.25577e-5 * height_m, 5.25588);
}

// A barometer sample at height_m: the height for the legacy build, the pressure under sensor model v2 [SEN-BAR-326].
inline EkfBaroInput baro_at(std::uint64_t time_usec, double height_m, double ground_m = 0.0)
{
    EkfBaroInput baro{time_usec, height_m};
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    baro.pressure_pa = isa_pressure_pa(ground_m + height_m);
#else
    (void)ground_m;
#endif
    return baro;
}

// The barometer reading the template controller config sets (ArduCopter 10 Hz, AP_Baro::calibrate() 10 + 5).
inline void configure_baro(ApEkf3Backend& ekf)
{
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    ekf.set_baro_processing(10.0, 10, 5);
#else
    (void)ekf;
#endif
}

}  // namespace hakoniwa::drone::control_adapter::test
