// SPDX-License-Identifier: GPL-3.0-or-later
// Ported from ArduPilot libraries/AP_Baro/AP_Baro_atmosphere.cpp (AP_Baro::get_altitude_from_pressure and the
// 1976 standard atmosphere table), so the plugin computes the barometer height as ArduPilot SITL does without
// building AP_Baro itself.
#include "ap_baro_atmosphere.hpp"

#include <cmath>
#include <limits>

namespace hakoniwa::drone::control_adapter {
namespace {
constexpr float kRadiusEarth = 6356.766E3f;  // m, 1976 model
constexpr float kRSpecific = 287.053072f;    // J/(kg K), 1976 model
constexpr float kGravity = 9.80665f;

struct Layer {
    float amsl_m, temp_K, pressure_Pa, density, temp_lapse;
};
constexpr Layer kLayers[] = {
    {-5000, 320.650f, 177687, 1.930467f, -6.5E-3f},
    {11000, 216.650f, 22632.1f, 0.363918f, 0},
    {20000, 216.650f, 5474.89f, 8.80349E-2f, 1E-3f},
    {32000, 228.650f, 868.019f, 1.32250E-2f, 2.8E-3f},
    {47000, 270.650f, 110.906f, 1.42753E-3f, 0},
    {51000, 270.650f, 66.9389f, 8.61606E-4f, -2.8E-3f},
    {71000, 214.650f, 3.95642f, 6.42110E-5f, -2.0E-3f},
    {84852, 186.946f, 0.37338f, 6.95788E-6f, 0},
};
constexpr unsigned kLayerCount = sizeof(kLayers) / sizeof(kLayers[0]);

unsigned layer_by_pressure(float pressure)
{
    for (unsigned idx = 1; idx < kLayerCount; idx++) {
        if (kLayers[idx].pressure_Pa < pressure) {
            return idx - 1;
        }
    }
    return kLayerCount - 1;
}
}  // namespace

float ap_altitude_from_pressure_1976(float pressure_pa)
{
    const unsigned idx = layer_by_pressure(pressure_pa);
    const float pressure_ratio = pressure_pa / kLayers[idx].pressure_Pa;
    if (!(pressure_ratio > 0.0f)) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    float alt;
    const float temp_slope = kLayers[idx].temp_lapse;
    if (temp_slope == 0.0f) {
        const float fac = -(kLayers[idx].temp_K * kRSpecific) / kGravity;
        alt = kLayers[idx].amsl_m + fac * logf(pressure_ratio);
    } else {
        const float fac = -(temp_slope * kRSpecific) / kGravity;
        alt = kLayers[idx].amsl_m + (kLayers[idx].temp_K / kLayers[idx].temp_lapse) * (powf(pressure_ratio, fac) - 1);
    }
    return (kRadiusEarth * alt) / (kRadiusEarth - alt);  // geopotential to geometric
}
}  // namespace hakoniwa::drone::control_adapter
