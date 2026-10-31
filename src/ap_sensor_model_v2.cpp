// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_sensor_model_v2.hpp"

#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
#include <AP_Math/AP_Math.h>

namespace hakoniwa::drone::control_adapter {
std::array<double, 3> ap_rotate_sensor_to_body(
    const ArdupilotSensorModelV2Config& config, const std::array<double, 3>& value)
{
    Vector3f result{static_cast<float>(value[0]), static_cast<float>(value[1]),
        static_cast<float>(value[2])};
    // [SEN-COM-350] Match ArduPilot's standard Rotation implementation.
    if (config.board_orientation != ROTATION_CUSTOM_1) {
        result.rotate(static_cast<Rotation>(config.board_orientation));
    } else {
        // [SEN-COM-320] Match Matrix3::from_euler(): Rz(yaw) * Ry(pitch) * Rx(roll).
        const float roll = radians(config.custom_rotation_deg[0]);
        const float pitch = radians(config.custom_rotation_deg[1]);
        const float yaw = radians(config.custom_rotation_deg[2]);
        const float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
        const float cy = cosf(yaw), sy = sinf(yaw);
        result = Vector3f{(cp * cy) * result.x + (sr * sp * cy - cr * sy) * result.y
                + (cr * sp * cy + sr * sy) * result.z,
            (cp * sy) * result.x + (sr * sp * sy + cr * cy) * result.y
                + (cr * sp * sy - sr * cy) * result.z,
            -sp * result.x + sr * cp * result.y + cr * cp * result.z};
    }
    return {result.x, result.y, result.z};
}

std::array<double, 3> ap_correct_mag_sensor_to_body(
    const ArdupilotSensorModelV2Config& config, const std::array<double, 3>& sensor_gauss)
{
    // [SEN-MAG-340] Match AP_Compass_Backend::correct_field(): S_c * (m + offsets).
    Vector3f mag{
        static_cast<float>(sensor_gauss[0] * 1000.0 + config.mag_offset_milligauss[0]),
        static_cast<float>(sensor_gauss[1] * 1000.0 + config.mag_offset_milligauss[1]),
        static_cast<float>(sensor_gauss[2] * 1000.0 + config.mag_offset_milligauss[2])};
    const Vector3f unscaled = mag;
    mag.x = config.mag_diagonal[0] * unscaled.x + config.mag_offdiagonal[0] * unscaled.y
        + config.mag_offdiagonal[1] * unscaled.z;
    mag.y = config.mag_offdiagonal[0] * unscaled.x + config.mag_diagonal[1] * unscaled.y
        + config.mag_offdiagonal[2] * unscaled.z;
    mag.z = config.mag_offdiagonal[1] * unscaled.x + config.mag_offdiagonal[2] * unscaled.y
        + config.mag_diagonal[2] * unscaled.z;
    const auto body_milligauss = ap_rotate_sensor_to_body(config, {mag.x, mag.y, mag.z});
    return {body_milligauss[0] * 0.001, body_milligauss[1] * 0.001,
        body_milligauss[2] * 0.001};
}
}
#endif
