#pragma once

#include <hakoniwa/drone/control_adapter/ekf_adapter.hpp>

#include <memory>

namespace hakoniwa::drone::control_adapter {

/**
 * Minimal in-process ArduPilot 4.6.3 NavEKF3 backend.
 *
 * This backend deliberately uses the Replay/DAL boundary instead of real
 * sensor drivers, EEPROM, or an ArduCopter vehicle process. The first version
 * supports one IMU, one GPS, one barometer, and one magnetometer.
 */
class ApEkf3Backend final : public IEkfAdapter {
public:
    ApEkf3Backend();
    ~ApEkf3Backend() override;

    ApEkf3Backend(const ApEkf3Backend&) = delete;
    ApEkf3Backend& operator=(const ApEkf3Backend&) = delete;

    void reset() override;
    void set_config(const EkfAdapterConfig& config) override;
    void set_armed_status(bool armed) override;
    void set_in_air_status(bool in_air) override;
    void set_vehicle_at_rest(bool at_rest) override;

    void push_imu(const EkfImuInput& input, double dt_sec) override;
    void push_mag(const EkfMagInput& input) override;
    void push_baro(const EkfBaroInput& input) override;
    void push_gps(const EkfHilGpsInput& input) override;

    void update() override;
    EkfEstimatedState get_estimated_state() const override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace hakoniwa::drone::control_adapter
