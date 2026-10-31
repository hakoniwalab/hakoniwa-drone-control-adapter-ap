// SPDX-License-Identifier: GPL-3.0-or-later

#include <hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp>

#include <AP_DAL/AP_DAL.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
#include "hakoniwa/drone/control_adapter/ap_sensor_model_v2.hpp"
#include "ap_baro_atmosphere.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

class MinimalDalUtil final : public AP_HAL::Util {
public:
    void set_hw_rtc(std::uint64_t time_utc_usec) override
    {
        rtc_usec_ = time_utc_usec;
    }

    std::uint64_t get_hw_rtc() const override
    {
        return rtc_usec_;
    }

    std::uint32_t available_memory() override
    {
        return 1024U * 1024U;
    }

private:
    std::uint64_t rtc_usec_{0};
};

MinimalDalUtil minimal_util;

class MinimalDalHal final : public AP_HAL::HAL {
public:
    MinimalDalHal()
        : AP_HAL::HAL(
              nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
              nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
              nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &minimal_util,
              nullptr, nullptr, nullptr, nullptr)
    {
    }

    void run(int, char* const[], Callbacks*) const override {}
};

MinimalDalHal minimal_hal;

std::int32_t degrees_to_e7(double value)
{
    return static_cast<std::int32_t>(std::llround(value * 1.0e7));
}

std::int32_t metres_to_centimetres(double value)
{
    return static_cast<std::int32_t>(std::llround(value * 100.0));
}


float finite_float(double value, const char* field)
{
    if (!std::isfinite(value)) {
        throw std::invalid_argument(std::string("non-finite EKF input: ") + field);
    }
    return static_cast<float>(value);
}

}  // namespace

const AP_HAL::HAL& hal = minimal_hal;

namespace hakoniwa::drone::control_adapter {

class ApEkf3Backend::Impl {
public:
    Impl()
        : dal_(AP::dal())
        , ekf_(std::make_unique<NavEKF3>())
    {
    }

    void apply_frontend_parameters()
    {
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        if (baro_alt_noise_m_ > 0.0) {
            ekf_->set_standalone_baro_alt_noise(static_cast<float>(baro_alt_noise_m_));
        }
        ekf_->set_standalone_hgt_delay_ms(static_cast<int16_t>(hgt_delay_ms_));
#endif
    }

    void reset()
    {
        // The DAL singleton remains alive, but NavEKF3 owns and releases its
        // cores. Recreate the frontend so service-level reset has the same
        // observable state as a fresh adapter instance.
        ekf_ = std::make_unique<NavEKF3>();
        apply_frontend_parameters();
        initialised_ = false;
        state_ = {};
        current_time_usec_ = 0;
        last_imu_time_usec_ = 0;
        home_lat_ = 0;
        home_lon_ = 0;
        home_alt_ = 0;
        home_set_ = false;
        have_imu_ = false;
        imu_pending_ = false;
        have_mag_ = false;
        have_baro_ = false;
        baro_ground_set_ = false;
        baro_ground_alt_m_ = 0.0;
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        baro_have_latest_ = false;
        baro_next_update_usec_ = 0;
        baro_updates_ = 0;
        baro_ground_sum_pa_ = 0.0;
        baro_ground_count_ = 0;
        baro_calibrated_ = false;
#endif
        have_gps_ = false;
        armed_ = false;
        in_air_ = false;
        in_air_since_usec_ = 0;
        at_rest_ = true;
    }

    // A host samples its sensors from time 0, before they have a first value
    // (Hakoniwa: pressure altitude, GPS position 0, no gravity on the IMU).
    // Such a sample would become the barometer's ground level and the home,
    // so samples at time 0 are not used.
    static bool before_first_value(std::uint64_t time_usec) { return time_usec == 0; }

    void push_imu(const EkfImuInput& input, double dt_sec)
    {
        if (before_first_value(input.time_usec)) {
            return;
        }
        if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) {
            throw std::invalid_argument("EKF IMU dt must be finite and positive");
        }
        if (last_imu_time_usec_ != 0 && input.time_usec <= last_imu_time_usec_) {
            throw std::invalid_argument("EKF IMU timestamps must be strictly increasing");
        }

        imu_header_.loop_rate_hz = static_cast<std::uint16_t>(
            std::clamp(std::llround(1.0 / dt_sec), 1LL, 65535LL));
        imu_header_.loop_delta_t = static_cast<float>(dt_sec);
        imu_header_.first_usable_accel = 0;
        imu_header_.first_usable_gyro = 0;
        imu_header_.accel_count = 1;
        imu_header_.gyro_count = 1;

        imu_.instance = 0;
        imu_.use_accel = true;
        imu_.use_gyro = true;
        imu_.get_delta_velocity_ret = true;
        imu_.get_delta_angle_ret = true;
        imu_.delta_velocity_dt = static_cast<float>(dt_sec);
        imu_.delta_angle_dt = static_cast<float>(dt_sec);
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        const auto acceleration = ap_correct_accel_sensor_to_body(sensor_model_v2_,
            {input.xacc_mps2, input.yacc_mps2, input.zacc_mps2});
        const auto angular_rate = ap_rotate_sensor_to_body(sensor_model_v2_,
            {input.xgyro_rad_s, input.ygyro_rad_s, input.zgyro_rad_s});
#else
        const std::array<double, 3> acceleration{input.xacc_mps2, input.yacc_mps2, input.zacc_mps2};
        const std::array<double, 3> angular_rate{input.xgyro_rad_s, input.ygyro_rad_s, input.zgyro_rad_s};
#endif
        imu_.delta_velocity = Vector3f{finite_float(acceleration[0], "xacc") * imu_.delta_velocity_dt,
            finite_float(acceleration[1], "yacc") * imu_.delta_velocity_dt,
            finite_float(acceleration[2], "zacc") * imu_.delta_velocity_dt};
        imu_.delta_angle = Vector3f{finite_float(angular_rate[0], "xgyro") * imu_.delta_angle_dt,
            finite_float(angular_rate[1], "ygyro") * imu_.delta_angle_dt,
            finite_float(angular_rate[2], "zgyro") * imu_.delta_angle_dt};

        current_time_usec_ = input.time_usec;
        last_imu_time_usec_ = input.time_usec;
        have_imu_ = true;
        imu_pending_ = true;
    }

    void push_mag(const EkfMagInput& input)
    {
        if (before_first_value(input.time_usec)) {
            return;
        }
        mag_header_.declination = radians(static_cast<float>(config_.mag_declination_deg));
        mag_header_.available = true;
        mag_header_.count = 1;
        mag_header_.auto_declination_enabled = false;
        mag_header_.num_enabled = 1;
        mag_header_.learn_offsets_enabled = false;
        mag_header_.consistent = true;
        mag_header_.first_usable = 0;

        mag_.instance = 0;
        mag_.last_update_usec = static_cast<std::uint32_t>(input.time_usec);
        mag_.offsets.zero();
        // AP_DAL_Compass exposes the field in milligauss.
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        const auto mag_gauss = ap_correct_mag_sensor_to_body(sensor_model_v2_,
            {input.xmag_gauss, input.ymag_gauss, input.zmag_gauss});
#else
        const std::array<double, 3> mag_gauss{input.xmag_gauss, input.ymag_gauss, input.zmag_gauss};
#endif
        mag_.field = Vector3f{finite_float(mag_gauss[0] * 1000.0, "xmag"),
            finite_float(mag_gauss[1] * 1000.0, "ymag"),
            finite_float(mag_gauss[2] * 1000.0, "zmag")};
        mag_.use_for_yaw = true;
        mag_.healthy = true;
        mag_.have_scale_factor = false;
        have_mag_ = true;
    }

    void push_baro(const EkfBaroInput& input)
    {
        if (before_first_value(input.time_usec)) {
            return;
        }
        baro_header_.primary = 0;
        baro_header_.num_instances = 1;
        baro_.instance = 0;
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        // [SEN-BAR-326] As ArduCopter: keep the latest pressure (SITL's AP_Baro_SITL copies its most recent sample)
        // and read it every 1/baro_update_hz (update_altitude). AP_Baro::calibrate() waits warmup updates, then
        // averages the next samples into the ground pressure; the EKF gets no barometer before that. The height is
        // ArduPilot's 1976-atmosphere altitude of the pressure minus that of the ground pressure.
        baro_latest_pressure_pa_ = finite_float(input.pressure_pa, "baro pressure");
        baro_have_latest_ = true;
        const auto period_usec = static_cast<std::uint64_t>(1e6 / baro_update_hz_);
        if (input.time_usec < baro_next_update_usec_) {
            return;
        }
        baro_next_update_usec_ = input.time_usec + period_usec;
        ++baro_updates_;
        if (!baro_calibrated_) {
            if (baro_updates_ > baro_warmup_updates_) {
                baro_ground_sum_pa_ += baro_latest_pressure_pa_;
                ++baro_ground_count_;
            }
            if (baro_ground_count_ < baro_calibration_samples_) {
                return;
            }
            baro_ground_altitude_m_ = ap_altitude_from_pressure_1976(
                static_cast<float>(baro_ground_sum_pa_ / baro_ground_count_));
            baro_calibrated_ = true;
        }
        baro_header_.primary = 0;
        baro_header_.num_instances = 1;
        baro_.instance = 0;
        baro_.last_update_ms = static_cast<std::uint32_t>(input.time_usec / 1000ULL);
        baro_.altitude = ap_altitude_from_pressure_1976(static_cast<float>(baro_latest_pressure_pa_))
            - baro_ground_altitude_m_;
        baro_.healthy = true;
        have_baro_ = true;
        return;
#endif
        baro_.last_update_ms = static_cast<std::uint32_t>(input.time_usec / 1000ULL);
        // AP_Baro reports altitude relative to the ground level it calibrated
        // at start; the host's pressure altitude is absolute. The first sample
        // after a reset is that ground level.
        const double pressure_alt_m = finite_float(input.pressure_alt_m, "baro altitude");
        if (!baro_ground_set_) {
            baro_ground_alt_m_ = pressure_alt_m;
            baro_ground_set_ = true;
        }
        baro_.altitude = static_cast<float>(pressure_alt_m - baro_ground_alt_m_);
        baro_.healthy = true;
        have_baro_ = true;
    }

    void push_gps(const EkfHilGpsInput& exact_input)
    {
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        const EkfHilGpsInput input = ap_ublox_pvt_fix(exact_input);
#else
        const EkfHilGpsInput& input = exact_input;
#endif
        if (before_first_value(input.time_usec)) {
            return;
        }
        gps_header_.num_sensors = 1;
        gps_header_.primary_sensor = 0;

        gps_info_.instance = 0;
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
        gps_info_.antenna_offset = Vector3f{static_cast<float>(sensor_model_v2_.gps_position_body_m[0]),
            static_cast<float>(sensor_model_v2_.gps_position_body_m[1]),
            static_cast<float>(sensor_model_v2_.gps_position_body_m[2])};
#else
        gps_info_.antenna_offset.zero();
#endif
        gps_info_.lag_sec = static_cast<float>(gps_lag_sec_);
        gps_info_.have_vertical_velocity = true;
        gps_info_.horizontal_accuracy_returncode = true;
        gps_info_.vertical_accuracy_returncode = true;
        gps_info_.get_lag_returncode = true;
        gps_info_.speed_accuracy_returncode = true;
        gps_info_.gps_yaw_deg_returncode = false;
        gps_info_.status = static_cast<std::uint8_t>(std::clamp(input.fix_type, 0, 6));
        gps_info_.num_sats = static_cast<std::uint8_t>(
            std::clamp(input.satellites_visible, 0, 255));

        gps_.instance = 0;
        gps_.last_message_time_ms = static_cast<std::uint32_t>(input.time_usec / 1000ULL);
        gps_.velocity = Vector3f{
            finite_float(input.vn_mps, "gps vn"),
            finite_float(input.ve_mps, "gps ve"),
            finite_float(input.vd_mps, "gps vd")};
        gps_.sacc = finite_float(input.sacc_mps, "gps sacc");
        gps_.yaw_deg = 0.0f;
        gps_.yaw_accuracy_deg = 0.0f;
        gps_.yaw_deg_time_ms = 0;
        gps_.lat = degrees_to_e7(input.lat_deg);
        gps_.lng = degrees_to_e7(input.lon_deg);
        gps_.alt = metres_to_centimetres(input.alt_m);
        gps_.hacc = finite_float(input.eph_m, "gps eph");
        gps_.vacc = finite_float(input.epv_m, "gps epv");
        gps_.hdop = static_cast<std::uint16_t>(std::clamp(
            std::llround(input.eph_m * 100.0), 0LL, 65535LL));

        if (!home_set_ && input.fix_type >= 3) {
            home_lat_ = gps_.lat;
            home_lon_ = gps_.lng;
            home_alt_ = gps_.alt;
            home_set_ = true;
        }
        have_gps_ = true;
    }

    void update()
    {
        // NavEKF3 runs once per new IMU sample: a host may call update() more
        // often than it pushes IMU data, and replaying the same delta angle /
        // velocity would integrate it twice. Mag, baro and GPS pushed meanwhile
        // go in with the next IMU sample.
        if (!have_imu_ || !imu_pending_) {
            return;
        }
        imu_pending_ = false;

        log_RFRH frame_header{};
        frame_header.time_us = current_time_usec_;
        frame_header.time_flying_ms = in_air_
            ? static_cast<std::uint32_t>((current_time_usec_ - in_air_since_usec_) / 1000ULL)
            : 0;
        dal_.handle_message(frame_header);

        log_RFRN frame_state{};
        frame_state.lat = home_lat_;
        frame_state.lng = home_lon_;
        frame_state.alt = home_alt_;
        frame_state.EAS2TAS = 1.0f;
        frame_state.available_memory = 1024U * 1024U;
        frame_state.vehicle_class = static_cast<std::uint8_t>(AP_DAL::VehicleClass::COPTER);
        frame_state.ekf_type = 3;
        // Hakoniwa's normal flight-state path currently supplies in_air/at_rest
        // but not a separate arm transition.  NavEKF3's Copter flight detector
        // treats an unarmed vehicle as certainly on ground, so in_air must imply
        // armed when constructing the DAL replay frame.
        frame_state.armed = armed_ || in_air_;
        frame_state.fly_forward = false;
        frame_state.takeoff_expected = in_air_;
        frame_state.touchdown_expected = !in_air_ && !at_rest_;
        dal_.handle_message(frame_state);

        dal_.handle_message(imu_header_);
        dal_.handle_message(imu_);
        if (have_mag_) {
            dal_.handle_message(mag_header_);
            dal_.handle_message(mag_);
        }
        if (have_baro_) {
            dal_.handle_message(baro_header_);
            dal_.handle_message(baro_);
        }
        if (have_gps_) {
            dal_.handle_message(gps_header_);
            dal_.handle_message(gps_info_);
            dal_.handle_message(gps_);
        }

        if (!initialised_) {
            initialised_ = ekf_->InitialiseFilter();
        }
        if (initialised_) {
            ekf_->UpdateFilter();
        }
        update_output();
        write_debug_csv();
    }

    void set_debug_csv_path(const std::string& path)
    {
        debug_csv_.close();
        debug_csv_.clear();
        if (path.empty()) {
            return;
        }
        debug_csv_.open(path, std::ios::out | std::ios::trunc);
        if (!debug_csv_) {
            throw std::runtime_error("failed to open EKF debug CSV: " + path);
        }
        debug_csv_ << "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,GX,GY,GZ,"
                      "AX,AY,AZ,MN,ME,MD,MX,MY,MZ,"
                      "IVN,IVE,IVD,IPN,IPE,IPD,IMX,IMY,IMZ,IYAW,"
                      "SV,SP,SH,SM,YawVar,errRP,onGround,inFlight,atRest,magLearning\n";
        debug_csv_.flush();
    }

    void write_debug_csv()
    {
        if (!debug_csv_ || !initialised_) {
            return;
        }
        Vector3f euler{}, velocity{}, gyro_bias{}, accel_bias{}, mag_ned{}, mag_xyz{};
        Quaternion attitude{};
        Vector2f position_ne{}, offset{};
        float position_d = 0.0f;
        Vector3f velocity_innov{}, position_innov{}, mag_innov{}, mag_ratio{};
        float tas_innov = 0.0f, yaw_innov = 0.0f;
        float velocity_ratio = 0.0f, position_ratio = 0.0f, height_ratio = 0.0f, tas_ratio = 0.0f;
        ekf_->getQuaternion(attitude);
        attitude.to_euler(euler);
        ekf_->getVelNED(velocity);
        ekf_->getPosNE(position_ne);
        ekf_->getPosD(position_d);
        ekf_->getGyroBias(-1, gyro_bias);
        ekf_->getAccelBias(-1, accel_bias);
        ekf_->getMagNED(mag_ned);
        ekf_->getMagXYZ(mag_xyz);
        ekf_->getInnovations(velocity_innov, position_innov, mag_innov, tas_innov, yaw_innov);
        ekf_->getVariances(velocity_ratio, position_ratio, height_ratio, mag_ratio, tas_ratio, offset);
        const float magnetic_ratio = std::max({mag_ratio.x, mag_ratio.y, mag_ratio.z});
        // NavEKF3 has no public getter for inhibitMagStates.  In heading mode the
        // three magnetic innovations are all zero; once 3-axis fusion is active
        // at least one is populated.  Record that observable fusion mode.
        const bool mag_learning = mag_innov.x != 0.0f || mag_innov.y != 0.0f || mag_innov.z != 0.0f;
        const double unavailable = std::numeric_limits<double>::quiet_NaN();
        debug_csv_ << std::setprecision(9) << current_time_usec_ << ','
                   << degrees(euler.x) << ',' << degrees(euler.y) << ',' << degrees(euler.z) << ','
                   << velocity.x << ',' << velocity.y << ',' << velocity.z << ','
                   << position_ne.x << ',' << position_ne.y << ',' << position_d << ','
                   << degrees(gyro_bias.x) << ',' << degrees(gyro_bias.y) << ',' << degrees(gyro_bias.z) << ','
                   << accel_bias.x << ',' << accel_bias.y << ',' << accel_bias.z << ','
                   << mag_ned.x << ',' << mag_ned.y << ',' << mag_ned.z << ','
                   << mag_xyz.x << ',' << mag_xyz.y << ',' << mag_xyz.z << ','
                   << velocity_innov.x << ',' << velocity_innov.y << ',' << velocity_innov.z << ','
                   << position_innov.x << ',' << position_innov.y << ',' << position_innov.z << ','
                   << mag_innov.x << ',' << mag_innov.y << ',' << mag_innov.z << ',' << degrees(yaw_innov) << ','
                   << velocity_ratio << ',' << position_ratio << ',' << height_ratio << ',' << magnetic_ratio << ','
                   << unavailable << ',' << unavailable << ',' << (!in_air_) << ',' << in_air_ << ','
                   << at_rest_ << ',' << mag_learning << '\n';
    }

    void update_output()
    {
        state_.time_usec = current_time_usec_;
        if (!initialised_) {
            return;
        }

        Quaternion attitude;
        // NavEKF3's quaternion rotates body (FRD) into NED, the convention of
        // the public adapter interface (Contract Test CT-EKF-002..004, 007).
        ekf_->getQuaternion(attitude);
        state_.attitude_quaternion_wxyz = {
            attitude.q1, attitude.q2, attitude.q3, attitude.q4};

        Vector3f velocity;
        ekf_->getVelNED(velocity);
        state_.velocity_ned_mps = {velocity.x, velocity.y, velocity.z};

        Vector2f position_ne;
        float position_d = 0.0f;
        const bool have_ne = ekf_->getPosNE(position_ne);
        const bool have_d = ekf_->getPosD(position_d);
        state_.position_local_ned_m = {position_ne.x, position_ne.y, position_d};

        Location location;
        const bool have_location = ekf_->getLLH(location);
        state_.lat_deg = static_cast<double>(location.lat) * 1.0e-7;
        state_.lon_deg = static_cast<double>(location.lng) * 1.0e-7;
        state_.alt_m_amsl = static_cast<double>(location.alt) * 0.01;

        nav_filter_status status{};
        ekf_->getFilterStatus(status);
        state_.attitude_valid = status.flags.attitude;
        state_.local_position_valid = have_ne && have_d
            && status.flags.horiz_pos_rel && status.flags.vert_pos;
        state_.global_position_valid = have_location && status.flags.horiz_pos_abs;
        state_.active_horizontal_aiding_sources = status.flags.using_gps ? 1 : 0;
        state_.active_horizontal_position_aiding_sources = status.flags.using_gps ? 1 : 0;
        state_.active_horizontal_velocity_aiding_sources = status.flags.using_gps ? 1 : 0;
        state_.active_vertical_position_aiding_sources = status.flags.vert_pos ? 1 : 0;
        state_.active_vertical_velocity_aiding_sources = status.flags.vert_vel ? 1 : 0;
        state_.gnss_pos_fused = status.flags.using_gps && status.flags.horiz_pos_abs;
        state_.gnss_vel_fused = status.flags.using_gps && status.flags.horiz_vel;
        state_.gps_hgt_fused = false;

        float velocity_ratio = 0.0f;
        float position_ratio = 0.0f;
        float height_ratio = 0.0f;
        float tas_ratio = 0.0f;
        Vector3f magnetic_ratio;
        Vector2f offset;
        if (ekf_->getVariances(
                velocity_ratio,
                position_ratio,
                height_ratio,
                magnetic_ratio,
                tas_ratio,
                offset)) {
            state_.horizontal_velocity_innovation_test_ratio = velocity_ratio;
            state_.vertical_velocity_innovation_test_ratio = velocity_ratio;
            state_.horizontal_position_innovation_test_ratio = position_ratio;
            state_.vertical_position_innovation_test_ratio = height_ratio;
        }
    }

    AP_DAL& dal_;
    std::unique_ptr<NavEKF3> ekf_;
    EkfAdapterConfig config_{};
    double gps_lag_sec_{0.0};
    EkfEstimatedState state_{};

    log_RISH imu_header_{};
    log_RISI imu_{};
    log_RMGH mag_header_{};
    log_RMGI mag_{};
    log_RBRH baro_header_{};
    log_RBRI baro_{};
    log_RGPH gps_header_{};
    log_RGPI gps_info_{};
    log_RGPJ gps_{};

    std::uint64_t current_time_usec_{0};
    std::uint64_t last_imu_time_usec_{0};
    std::int32_t home_lat_{0};
    std::int32_t home_lon_{0};
    std::int32_t home_alt_{0};
    bool home_set_{false};
    bool have_imu_{false};
    bool imu_pending_{false};
    bool baro_ground_set_{false};
    double baro_ground_alt_m_{0.0};
    bool have_mag_{false};
    bool have_baro_{false};
    bool have_gps_{false};
    bool initialised_{false};
    bool armed_{false};
    bool in_air_{false};
    std::uint64_t in_air_since_usec_{0};
    bool at_rest_{true};
    std::ofstream debug_csv_{};
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
    ArdupilotSensorModelV2Config sensor_model_v2_{};
    double baro_alt_noise_m_{0.0};
    double hgt_delay_ms_{60.0};
    double baro_update_hz_{0.0};
    int baro_warmup_updates_{0};
    int baro_calibration_samples_{0};
    double baro_latest_pressure_pa_{0.0};
    bool baro_have_latest_{false};
    std::uint64_t baro_next_update_usec_{0};
    int baro_updates_{0};
    double baro_ground_sum_pa_{0.0};
    int baro_ground_count_{0};
    float baro_ground_altitude_m_{0.0f};
    bool baro_calibrated_{false};
#endif
};

ApEkf3Backend::ApEkf3Backend()
    : impl_(std::make_unique<Impl>())
{
}

ApEkf3Backend::~ApEkf3Backend() = default;

void ApEkf3Backend::reset() { impl_->reset(); }
void ApEkf3Backend::set_config(const EkfAdapterConfig& config) { impl_->config_ = config; }
void ApEkf3Backend::set_armed_status(bool armed) { impl_->armed_ = armed; }
void ApEkf3Backend::set_in_air_status(bool in_air)
{
    if (in_air && !impl_->in_air_) {
        impl_->in_air_since_usec_ = impl_->current_time_usec_;
    } else if (!in_air) {
        impl_->in_air_since_usec_ = 0;
    }
    impl_->in_air_ = in_air;
}
void ApEkf3Backend::set_vehicle_at_rest(bool at_rest) { impl_->at_rest_ = at_rest; }
void ApEkf3Backend::set_gps_lag_sec(double lag_sec)
{
    if (!std::isfinite(lag_sec) || lag_sec < 0.0 || lag_sec > 0.25) {
        throw std::invalid_argument("GPS lag must be 0..0.25 s (NavEKF3 limit)");
    }
    impl_->gps_lag_sec_ = lag_sec;
}
void ApEkf3Backend::set_debug_csv_path(const std::string& path) { impl_->set_debug_csv_path(path); }
#ifdef HAKO_ENABLE_SENSOR_MODEL_V2
void ApEkf3Backend::set_sensor_model_v2_config(const ArdupilotSensorModelV2Config& config)
{
    impl_->sensor_model_v2_ = config;
    // [SEN-COM-350] NavEKF3 reads INS_POS1 through AP_DAL_InertialSensor.
    impl_->dal_.ins().set_imu_pos_offset(Vector3f{static_cast<float>(config.imu_position_body_m[0]),
        static_cast<float>(config.imu_position_body_m[1]),
        static_cast<float>(config.imu_position_body_m[2])});
}
void ApEkf3Backend::set_baro_expectations(double alt_noise_m, double hgt_delay_ms)
{
    impl_->baro_alt_noise_m_ = alt_noise_m;
    impl_->hgt_delay_ms_ = hgt_delay_ms;
    impl_->apply_frontend_parameters();
}
void ApEkf3Backend::set_baro_processing(double update_hz, int warmup_updates, int samples)
{
    impl_->baro_update_hz_ = update_hz;
    impl_->baro_warmup_updates_ = warmup_updates;
    impl_->baro_calibration_samples_ = samples;
}
#endif
void ApEkf3Backend::push_imu(const EkfImuInput& input, double dt_sec) { impl_->push_imu(input, dt_sec); }
void ApEkf3Backend::push_mag(const EkfMagInput& input) { impl_->push_mag(input); }
void ApEkf3Backend::push_baro(const EkfBaroInput& input) { impl_->push_baro(input); }
void ApEkf3Backend::push_gps(const EkfHilGpsInput& input) { impl_->push_gps(input); }
void ApEkf3Backend::update() { impl_->update(); }
EkfEstimatedState ApEkf3Backend::get_estimated_state() const { return impl_->state_; }

}  // namespace hakoniwa::drone::control_adapter

int AP_HAL::Util::vsnprintf(char*, std::size_t, const char*, va_list)
{
    return -1;
}

void* no_logger = nullptr;

AP_Logger& AP::logger()
{
    return *static_cast<AP_Logger*>(no_logger);
}

void AP_Logger::WriteBlock(const void*, std::uint16_t)
{
}
