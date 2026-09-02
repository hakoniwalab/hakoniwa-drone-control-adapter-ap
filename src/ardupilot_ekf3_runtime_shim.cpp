// SPDX-License-Identifier: GPL-3.0-or-later

#include <AP_Math/AP_Math.h>
#include <AP_HAL/Util.h>
#include <AP_Param/AP_Param.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>

void AP_Param::setup_object_defaults(const void* object_pointer, const GroupInfo* group_info)
{
    const auto base = reinterpret_cast<std::uintptr_t>(object_pointer);
    for (std::size_t i = 0; group_info[i].type != AP_PARAM_NONE; ++i) {
        const auto& info = group_info[i];
        auto* ptr = reinterpret_cast<void*>(base + info.offset);
        float value = info.def_value;
        if ((info.flags & AP_PARAM_FLAG_DEFAULT_POINTER) != 0) {
            const auto* default_param = reinterpret_cast<const AP_Float*>(
                base + info.offset - info.def_value_offset);
            value = default_param->get();
        }
        switch (info.type) {
        case AP_PARAM_INT8:
            static_cast<AP_Int8*>(ptr)->set(static_cast<std::int8_t>(value));
            break;
        case AP_PARAM_INT16:
            static_cast<AP_Int16*>(ptr)->set(static_cast<std::int16_t>(value));
            break;
        case AP_PARAM_INT32:
            static_cast<AP_Int32*>(ptr)->set(static_cast<std::int32_t>(value));
            break;
        case AP_PARAM_FLOAT:
            static_cast<AP_Float*>(ptr)->set(value);
            break;
        case AP_PARAM_VECTOR3F:
            static_cast<AP_Vector3f*>(ptr)->set(Vector3f{value, value, value});
            break;
        default:
            break;
        }
    }
}

template<typename T, ap_var_type PT>
void AP_ParamT<T, PT>::set_and_default(const T& value)
{
    set(value);
}

template class AP_ParamT<std::int8_t, AP_PARAM_INT8>;

extern "C" void* __real_malloc(std::size_t size)
{
    return std::malloc(size);
}

void AP_HAL::Util::set_soft_armed(const bool armed)
{
    soft_armed = armed;
    last_armed_change_ms = 0;
}
