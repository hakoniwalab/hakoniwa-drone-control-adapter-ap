// SPDX-License-Identifier: GPL-3.0-or-later

// AC_PID is reused directly from ArduPilot, but this adapter is not an
// ArduPilot vehicle process. It therefore has no HAL clock or EEPROM-backed
// AP_Param store. These narrow definitions provide only the runtime contract
// needed by AC_PID; controller values are supplied explicitly by the adapter.
// The DAL archive used by this project omits AP_HAL_SITL/system.cpp.o: that
// object requires a complete SITL HAL which this adapter deliberately does
// not initialize.  These implementations are the sole definitions of its
// public system API and use the adapter's standalone monotonic clock.

#include <AP_HAL/system.h>
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Param/AP_Param.h>

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <type_traits>

#if !defined(HAKONIWA_AP_STANDALONE_MATH_ONLY)
// Weak: the control stages set their values from their own configuration, but
// NavEKF3 needs its parameter defaults. When the EKF3 runtime shim is linked in
// as well (the Control Link plugin), its implementation must be the one used.
__attribute__((weak)) void AP_Param::setup_object_defaults(const void*, const GroupInfo*)
{
}

bool AP_Param::load()
{
    return false;
}

void AP_Param::save(bool)
{
}
#endif

// Non-EKF control code reports invalid arguments through INTERNAL_ERROR; this
// adapter has no vehicle to flag, so its standalone runtime records nothing.
#if !defined(HAKONIWA_AP_DAL_MATH_LINKED)
__attribute__((weak)) void AP_InternalError::error(const AP_InternalError::error_t, uint16_t)
{
}

namespace AP {
__attribute__((weak)) AP_InternalError& internalerror()
{
    static AP_InternalError instance;
    return instance;
}
}  // namespace AP
#endif

#if !defined(HAKONIWA_AP_STANDALONE_MATH_ONLY)
namespace AP_HAL {

namespace {
uint64_t elapsed_micros()
{
    using Clock = std::chrono::steady_clock;
    static const auto epoch = Clock::now();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now() - epoch).count());
}
}  // namespace

void init()
{
}

void panic(const char* errormsg, ...)
{
    std::va_list args;
    va_start(args, errormsg);
    std::vfprintf(stderr, errormsg, args);
    va_end(args);
    std::fputc('\n', stderr);
    std::abort();
}

uint32_t micros()
{
    return static_cast<uint32_t>(micros64());
}

uint32_t millis()
{
    return static_cast<uint32_t>(millis64());
}

uint16_t millis16()
{
    return static_cast<uint16_t>(millis64());
}

uint64_t micros64()
{
    return elapsed_micros();
}

uint64_t millis64()
{
    return micros64() / 1000U;
}

void dump_stack_trace()
{
}

void dump_core_file()
{
}

}  // namespace AP_HAL
#endif

#if !defined(HAKONIWA_AP_DAL_MATH_LINKED)
// Non-EKF builds do not link AP_Math.cpp.o, so retain its matching low-pass
// helper here.  EKF builds receive the sole definition from the DAL archive.
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
#endif

// Non-EKF builds do not link DAL's AP_Math/vector3.cpp.o or AP_Math.cpp.o.
// Keep their standalone math support here only in that configuration.
#if !defined(HAKONIWA_AP_DAL_MATH_LINKED)
// AP_Math/vector3.cpp's Vector3<float>::length, which AP_Math/control.cpp (kinematic_limit) uses.
template <>
float Vector3<float>::length() const
{
    return std::sqrt(x * x + y * y + z * z);
}

// sqrt_controller, inv_sqrt_controller and the input shaping (shape_pos_vel_accel*,
// update_pos_vel_accel*) come from ArduPilot's own AP_Math/control.cpp, linked in.

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

template <typename T>
float safe_sqrt(const T value)
{
    const float result = std::sqrt(static_cast<float>(value));
    return std::isnan(result) ? 0.0f : result;
}

template float safe_sqrt<float>(float);
template float safe_sqrt<double>(double);

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
#endif
