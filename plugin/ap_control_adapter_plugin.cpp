// SPDX-License-Identifier: GPL-3.0-or-later
//
// The ArduPilot adapter as a Control Link plugin (hakoniwa-drone-control-adapter
// adapter_plugin.hpp): a host loads this library at run time with
// controller.backendType adapter-plugin; it is not linked into the host.

#include "hakoniwa/drone/control_adapter/adapter_plugin.hpp"
#include "hakoniwa/drone/control_adapter/ap_allocation_feedback_policy.hpp"
#include "hakoniwa/drone/control_adapter/ap_position_control_3d_backend.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include "ap_sensor_filters.hpp"
#ifdef HAKO_AP_PLUGIN_WITH_EKF3
#include "hakoniwa/drone/control_adapter/ap_ekf3_backend.hpp"
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

namespace adapter = hakoniwa::drone::control_adapter;
namespace filters = hakoniwa::drone::control_adapter::plugin_filters;
using adapter::plugin::BackendKind;

constexpr double kPi = 3.14159265358979323846;

struct Context {
    adapter::ArdupilotControllerConfig config;
    std::string capabilities;
    // AP_Motors limit.throttle_lower/upper: written by this context's allocation, read on the next
    // cycle by its altitude stages (AC_PosControl::update_z_controller). Shared so a stage keeps it
    // whatever the destruction order.
    std::shared_ptr<adapter::ApMotorThrottleLimits> motor_limits{std::make_shared<adapter::ApMotorThrottleLimits>()};
};

void set_error(char* error, std::size_t error_size, const std::string& message)
{
    if (error != nullptr && error_size > 0) {
        std::snprintf(error, error_size, "%s", message.c_str());
    }
}

void clear_error(char* error, std::size_t error_size)
{
    if (error != nullptr && error_size > 0) {
        error[0] = '\0';
    }
}

std::string read_file(const std::string& path)
{
    std::ifstream stream(path);
    std::ostringstream text;
    text << stream.rdbuf();
    return text.str();
}

// "KEY VALUE" lines of the Hakoniwa controller parameter text ('#' comments).
std::map<std::string, double> parse_params(const std::string& text)
{
    std::map<std::string, double> params;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (const auto comment = line.find('#'); comment != std::string::npos) {
            line.erase(comment);
        }
        std::istringstream fields(line);
        std::string key;
        double value = 0.0;
        if (fields >> key >> value) {
            params[key] = value;
        }
    }
    return params;
}

// The Hakoniwa PID_* parameters onto the ArduPilot controller configuration:
// the same stages the PX4 adapter's sync maps (docs/ardupilot-controller-config.md).
// A parameter the text does not set keeps the configuration file's value.
void apply_params(adapter::ArdupilotControllerConfig& config, const std::map<std::string, double>& params)
{
    const auto set = [&](const char* key, double& target) {
        if (const auto it = params.find(key); it != params.end()) {
            target = it->second;
        }
    };
    const auto set_same = [&](const char* x_key, const char* y_key, double& target) {
        const auto x = params.find(x_key);
        const auto y = params.find(y_key);
        if (x == params.end() && y == params.end()) {
            return;
        }
        if (x == params.end() || y == params.end() || std::abs(x->second - y->second) > 1e-9) {
            throw std::runtime_error(std::string(x_key) + " and " + y_key
                + " must be equal: ArduPilot has one horizontal gain for both axes");
        }
        target = x->second;
    };

    auto& rate = config.rate_control;
    set("PID_ROLL_RATE_Kp", rate.roll.p);
    set("PID_ROLL_RATE_Ki", rate.roll.i);
    set("PID_ROLL_RATE_Kd", rate.roll.d);
    set("PID_PITCH_RATE_Kp", rate.pitch.p);
    set("PID_PITCH_RATE_Ki", rate.pitch.i);
    set("PID_PITCH_RATE_Kd", rate.pitch.d);
    set("PID_YAW_RATE_Kp", rate.yaw.p);
    set("PID_YAW_RATE_Ki", rate.yaw.i);
    set("PID_YAW_RATE_Kd", rate.yaw.d);

    auto& attitude = config.attitude_control;
    set("PID_ROLL_Kp", attitude.angle_roll_p);
    set("PID_PITCH_Kp", attitude.angle_pitch_p);
    set("PID_YAW_Kp", attitude.angle_yaw_p);
    for (const auto& [key, target] : {
             std::pair{"PID_ROLL_RPM_MAX", &attitude.rate_roll_max_rad_sec},
             std::pair{"PID_PITCH_RPM_MAX", &attitude.rate_pitch_max_rad_sec},
             std::pair{"PID_YAW_RPM_MAX", &attitude.rate_yaw_max_rad_sec},
         }) {
        if (const auto it = params.find(key); it != params.end()) {
            *target = it->second * kPi / 30.0;
        }
    }

    auto& altitude = config.altitude_control;
    set("PID_ALT_Kp", altitude.position_p);
    set("PID_ALT_SPD_Kp", altitude.velocity_p);
    set("PID_ALT_SPD_Ki", altitude.velocity_i);
    set("PID_ALT_SPD_Kd", altitude.velocity_d);
    if (const auto it = params.find("PID_ALT_MAX_SPD"); it != params.end()) {
        altitude.speed_up_mps = it->second;
        altitude.speed_down_mps = it->second;
    }

    auto& horizontal = config.horizontal_control;
    set_same("PID_POS_X_Kp", "PID_POS_Y_Kp", horizontal.position_p);
    set_same("PID_POS_VX_Kp", "PID_POS_VY_Kp", horizontal.velocity_p);
    set_same("PID_POS_VX_Ki", "PID_POS_VY_Ki", horizontal.velocity_i);
    set_same("PID_POS_VX_Kd", "PID_POS_VY_Kd", horizontal.velocity_d);
    set("PID_POS_MAX_SPD", horizontal.speed_max_mps);
    const auto max_roll = params.find("PID_POS_MAX_ROLL");
    const auto max_pitch = params.find("PID_POS_MAX_PITCH");
    if (max_roll != params.end() && max_pitch != params.end()) {
        horizontal.angle_max_rad = std::min(max_roll->second, max_pitch->second) * kPi / 180.0;
    }
}

void* create_context(const char* config_path, char* error, std::size_t error_size)
{
    clear_error(error, error_size);
    try {
        auto* context = new Context{};
        context->config = adapter::ArdupilotControllerConfigLoader{}.load_from_file(config_path);
        context->capabilities = read_file(HAKO_AP_CAPABILITY_PATH);
        return context;
    } catch (const std::exception& exception) {
        set_error(error, error_size, exception.what());
        return nullptr;
    }
}

void destroy_context(void* context)
{
    delete static_cast<Context*>(context);
}

bool apply_hakoniwa_params(void* context, const char* param_text, char* error, std::size_t error_size)
{
    clear_error(error, error_size);
    try {
        apply_params(static_cast<Context*>(context)->config, parse_params(param_text != nullptr ? param_text : ""));
        return true;
    } catch (const std::exception& exception) {
        set_error(error, error_size, exception.what());
        return false;
    }
}

void* create_backend(void* context, BackendKind kind, const void* argument, char* error, std::size_t error_size)
{
    (void)argument;
    clear_error(error, error_size);
    auto* const plugin_context = static_cast<Context*>(context);
    const auto& config = plugin_context->config;
    const std::shared_ptr<const adapter::ApMotorThrottleLimits> motor_limits = plugin_context->motor_limits;
    try {
        switch (kind) {
        // The sensor filters (INS_GYRO_FILTER, INS_ACCEL_FILTER) wrap the stages
        // that read the IMU, as AP_InertialSensor does before ArduPilot's controllers.
        case BackendKind::RateControl:
            if (config.sensor_filter.gyro_cutoff_hz > 0.0) {
                return static_cast<adapter::IRateControlBackend*>(new filters::FilteredRateControl(config));
            }
            return static_cast<adapter::IRateControlBackend*>(new adapter::ApRateControlBackend(config.rate_control));
        case BackendKind::AttitudeControl:
            return static_cast<adapter::IAttitudeControlBackend*>(
                new adapter::ApAttitudeControlBackend(config.attitude_control));
        case BackendKind::AltitudeControl:
            if (config.sensor_filter.accel_cutoff_hz > 0.0) {
                auto* backend = new filters::FilteredAltitudeControl(config);
                backend->set_motor_limits_source(motor_limits);
                return static_cast<adapter::IAltitudeControlBackend*>(backend);
            } else {
                auto* backend = new adapter::ApAltitudeControlBackend(config.altitude_control);
                backend->set_motor_limits_source(motor_limits);
                return static_cast<adapter::IAltitudeControlBackend*>(backend);
            }
        case BackendKind::HorizontalPositionControl:
            return static_cast<adapter::IHorizontalPositionControlBackend*>(
                new adapter::ApHorizontalPositionControlBackend(config.horizontal_control));
        case BackendKind::ControlAllocation: {
            auto* backend = new adapter::ApControlAllocationBackend(config.control_allocation);
            backend->set_motor_limits_sink(plugin_context->motor_limits);
            return static_cast<adapter::IControlAllocationBackend*>(backend);
        }
        case BackendKind::AllocationFeedbackPolicy:
            return static_cast<adapter::IAllocationFeedbackPolicy*>(new adapter::ApAllocationFeedbackPolicy());
        case BackendKind::Ekf: {
#ifdef HAKO_AP_PLUGIN_WITH_EKF3
            // NavEKF3 through the DAL replay boundary (ap_ekf3_backend.hpp).
            auto* ekf = new adapter::ApEkf3Backend();
            ekf->set_gps_lag_sec(config.ekf.gps_delay_ms * 1.0e-3);
            if (argument != nullptr) {
                ekf->set_config(*static_cast<const adapter::EkfAdapterConfig*>(argument));
            }
            return static_cast<adapter::IEkfAdapter*>(ekf);
#else
            return nullptr;  // built without NavEKF3 (HAKONIWA_AP_ENABLE_EKF3=OFF)
#endif
        }
        case BackendKind::PositionControl3D:
            if (config.sensor_filter.accel_cutoff_hz > 0.0) {
                auto* backend = new filters::FilteredPositionControl3D(config);
                backend->set_motor_limits_source(motor_limits);
                return static_cast<adapter::IPositionControl3DBackend*>(backend);
            } else {
                auto* backend = new adapter::ApPositionControl3DBackend(config.altitude_control, config.horizontal_control,
                                                                      adapter::position_shaping_for(config));
                backend->set_motor_limits_source(motor_limits);
                return static_cast<adapter::IPositionControl3DBackend*>(backend);
            }
        }
        set_error(error, error_size, "unknown backend kind " + std::to_string(static_cast<unsigned>(kind)));
        return nullptr;
    } catch (const std::exception& exception) {
        set_error(error, error_size, exception.what());
        return nullptr;
    }
}

void destroy_backend(BackendKind kind, void* backend)
{
    switch (kind) {
    case BackendKind::RateControl:
        delete static_cast<adapter::IRateControlBackend*>(backend);
        break;
    case BackendKind::AttitudeControl:
        delete static_cast<adapter::IAttitudeControlBackend*>(backend);
        break;
    case BackendKind::AltitudeControl:
        delete static_cast<adapter::IAltitudeControlBackend*>(backend);
        break;
    case BackendKind::HorizontalPositionControl:
        delete static_cast<adapter::IHorizontalPositionControlBackend*>(backend);
        break;
    case BackendKind::PositionControl3D:
        delete static_cast<adapter::IPositionControl3DBackend*>(backend);
        break;
    case BackendKind::ControlAllocation:
        delete static_cast<adapter::IControlAllocationBackend*>(backend);
        break;
    case BackendKind::AllocationFeedbackPolicy:
        delete static_cast<adapter::IAllocationFeedbackPolicy*>(backend);
        break;
    case BackendKind::Ekf:
        delete static_cast<adapter::IEkfAdapter*>(backend);
        break;
    }
}

const char* capabilities_json(void* context)
{
    return static_cast<Context*>(context)->capabilities.c_str();
}

const adapter::plugin::ControlAdapterPluginV1 kPlugin{
    adapter::plugin::kPluginAbiVersion,
    adapter::plugin::kInterfaceRevision,
    "ardupilot",
    create_context,
    destroy_context,
    apply_hakoniwa_params,
    create_backend,
    destroy_backend,
    capabilities_json,
};

}  // namespace

extern "C" __attribute__((visibility("default")))
const hakoniwa::drone::control_adapter::plugin::ControlAdapterPluginV1* hako_control_adapter_plugin_v1()
{
    return &kPlugin;
}
