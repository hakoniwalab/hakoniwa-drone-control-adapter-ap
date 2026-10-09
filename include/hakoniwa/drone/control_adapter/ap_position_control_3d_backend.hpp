// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "hakoniwa/drone/control_adapter/ap_altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ap_horizontal_position_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"

namespace hakoniwa::drone::control_adapter {

/**
 * ArduPilot's 3D position control: AC_PosControl runs its z and xy controllers
 * together, which here are the altitude (AC_P_1D/AC_PID) and horizontal
 * (AC_P_2D/AC_PID_2D) stages of this adapter. The stage turns their collective
 * thrust and lean angles, with the yaw target, into the target attitude.
 */
class ApPositionControl3DBackend final : public IPositionControl3DBackend {
public:
    ApPositionControl3DBackend(
        const ApAltitudeControlBackendConfig& altitude,
        const ApHorizontalPositionControlBackendConfig& horizontal);

    void reset() override;
    PositionControl3DOutput run_position(const PositionControl3DPositionInput& input, double dt_sec) override;
    PositionControl3DOutput run_velocity(const PositionControl3DVelocityInput& input, double dt_sec) override;

private:
    PositionControl3DOutput run(
        const PositionControl3DState& state,
        AltitudeControlInput altitude,
        HorizontalPositionControlInput horizontal,
        const std::optional<double>& target_yaw_rad,
        const std::optional<double>& target_yaw_rate_rad_sec,
        double dt_sec);

    ApAltitudeControlBackend altitude_;
    ApHorizontalPositionControlBackend horizontal_;
};

}  // namespace hakoniwa::drone::control_adapter
