// SPDX-License-Identifier: GPL-3.0-or-later

// AC_PID is reused directly from ArduPilot, but this adapter is not an
// ArduPilot vehicle process. It therefore has no HAL clock or EEPROM-backed
// AP_Param store. These narrow definitions provide only the runtime contract
// needed by AC_PID; controller values are supplied explicitly by the adapter.
// The math helper behavior below follows ArduPilot 4.6.3 AP_Math.cpp.

#include <AP_HAL/system.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <type_traits>

void AP_Param::setup_object_defaults(const void*, const GroupInfo*)
{
}

bool AP_Param::load()
{
    return false;
}

void AP_Param::save(bool)
{
}

namespace AP_HAL {

uint32_t millis()
{
    using Clock = std::chrono::steady_clock;
    static const auto epoch = Clock::now();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - epoch)
            .count());
}

}  // namespace AP_HAL

float calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (dt < 0.0f || cutoff_freq < 0.0f) {
        return 1.0f;
    }
    if (cutoff_freq == 0.0f) {
        return 1.0f;
    }
    if (dt == 0.0f) {
        return 0.0f;
    }
    const float rc = 1.0f / (M_2PI * cutoff_freq);
    return dt / (dt + rc);
}

template <typename T>
T constrain_value_line(const T value, const T low, const T high, uint32_t)
{
    if (std::isnan(value)) {
        return (low + high) / 2;
    }
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

template float constrain_value_line<float>(float, float, float, uint32_t);
template double constrain_value_line<double>(double, double, double, uint32_t);

template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_integral<
    typename std::common_type<Arithmetic1, Arithmetic2>::type>::value,
    bool>::type
is_equal(const Arithmetic1 lhs, const Arithmetic2 rhs)
{
    using Common = typename std::common_type<Arithmetic1, Arithmetic2>::type;
    return static_cast<Common>(lhs) == static_cast<Common>(rhs);
}

template <typename Arithmetic1, typename Arithmetic2>
typename std::enable_if<std::is_floating_point<
    typename std::common_type<Arithmetic1, Arithmetic2>::type>::value,
    bool>::type
is_equal(const Arithmetic1 lhs, const Arithmetic2 rhs)
{
    using Common = typename std::common_type<Arithmetic1, Arithmetic2>::type;
    return std::abs(static_cast<Common>(lhs - rhs))
        < std::numeric_limits<Common>::epsilon();
}

template bool is_equal<int>(int, int);
template bool is_equal<short>(short, short);
template bool is_equal<long>(long, long);
template bool is_equal<float>(float, float);
template bool is_equal<double>(double, double);
