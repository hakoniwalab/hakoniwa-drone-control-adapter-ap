// SPDX-License-Identifier: GPL-3.0-or-later

#include "hakoniwa/drone/control_adapter/adapter_plugin.hpp"
#include "hakoniwa/drone/control_adapter/altitude_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/position_control_3d_backend.hpp"
#include "hakoniwa/drone/control_adapter/rate_control_backend.hpp"
#include "hakoniwa/drone/control_adapter/ardupilot_controller_config_loader.hpp"
#include "../plugin/ap_sensor_filters.hpp"

#include <cmath>
#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace adapter = hakoniwa::drone::control_adapter;
namespace filters = hakoniwa::drone::control_adapter::plugin_filters;
namespace plugin = hakoniwa::drone::control_adapter::plugin;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void biquad_matches_ardupilot_response()
{
    filters::Biquad2p filter;
    filter.configure(333.33, 20.0);
    require(std::abs(filter.apply(0.0)) < 1e-15, "zero initial sample");
    // An impulse after zero initialisation exposes DigitalBiquadFilter's b0.
    const double h0 = filter.apply(1.0);
    const double h1 = filter.apply(0.0);
    const double h2 = filter.apply(0.0);
    const double a1 = (2.0 * h0 - h1) / h0;
    const double a2 = (h0 - a1 * h1 - h2) / h0;
    require(std::abs(h0 - 0.02786) < 5e-5, "ArduPilot b0 coefficient");
    require(std::abs(a1 + 1.4755) < 5e-4, "ArduPilot a1 coefficient");
    require(std::abs(a2 - 0.5869) < 5e-4, "ArduPilot a2 coefficient");

    filters::Biquad2p passthrough;
    passthrough.configure(333.33, 0.0);
    require(passthrough.apply(12.5) == 12.5, "zero cutoff must pass through");

    filters::Biquad2p clamped, forty;
    clamped.configure(100.0, 90.0);
    forty.configure(100.0, 40.0);
    for (double sample : {0.0, 1.0, -1.0, 0.5}) {
        require(std::abs(clamped.apply(sample) - forty.apply(sample)) < 1e-12,
            "cutoff must clamp to sample rate times 0.4");
    }

    filters::Biquad2p steady;
    steady.configure(333.33, 20.0);
    require(std::abs(steady.apply(3.0) - 3.0) < 1e-12,
        "first sample must initialise at steady state");
    (void)steady.apply(0.0);
    steady.reset();
    require(std::abs(steady.apply(-2.0) + 2.0) < 1e-12,
        "reset must reinitialise from the next sample");
}

std::string read_file(const char* path)
{
    std::ifstream stream(path);
    std::ostringstream text;
    text << stream.rdbuf();
    return text.str();
}

std::string filtered_config(const char* source_path)
{
    std::string text = read_file(source_path);
    const std::string marker = "\"parameters\": {";
    const auto position = text.find(marker);
    require(position != std::string::npos, "config parameters object");
    text.insert(position + marker.size(),
        "\n    \"INS_GYRO_FILTER\": 5, \"INS_ACCEL_FILTER\": 5,");
    const std::string path = "/tmp/hako-ap-sensor-filter-test.json";
    std::ofstream(path) << text;
    return path;
}

void loader_defaults_filters_to_zero(const char* config_path)
{
    adapter::ArdupilotControllerConfigLoader loader;
    const auto plain = loader.load_from_text(read_file(config_path));
    require(plain.sensor_filter.gyro_cutoff_hz == 0.0, "missing gyro filter defaults to zero");
    require(plain.sensor_filter.accel_cutoff_hz == 0.0, "missing accel filter defaults to zero");
    const auto configured = loader.load_from_text(
        "{\"INS_GYRO_FILTER\":20,\"INS_ACCEL_FILTER\":10," + read_file(config_path).substr(1));
    require(configured.sensor_filter.gyro_cutoff_hz == 20.0, "loads gyro cutoff");
    require(configured.sensor_filter.accel_cutoff_hz == 10.0, "loads accel cutoff");
}

struct LoadedPlugin {
    void* library{};
    const plugin::ControlAdapterPluginV1* api{};
    ~LoadedPlugin() { if (library) dlclose(library); }
};

LoadedPlugin load_plugin(const char* path)
{
    LoadedPlugin loaded;
    loaded.library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    require(loaded.library != nullptr, dlerror());
    auto entry = reinterpret_cast<HakoControlAdapterPluginEntryV1>(
        dlsym(loaded.library, HAKO_CONTROL_ADAPTER_PLUGIN_ENTRY_V1));
    require(entry != nullptr, "plugin entry symbol");
    loaded.api = entry();
    return loaded;
}

template<class Interface>
Interface* backend(const LoadedPlugin& loaded, void* context, plugin::BackendKind kind)
{
    char error[256]{};
    auto* result = static_cast<Interface*>(loaded.api->create_backend(context, kind, nullptr, error, sizeof(error)));
    require(result != nullptr, error);
    return result;
}

void plugin_filters_reduce_high_frequency_inputs(const char* library_path, const char* config_path)
{
    auto loaded = load_plugin(library_path);
    char error[256]{};
    void* plain_context = loaded.api->create_context(config_path, error, sizeof(error));
    require(plain_context != nullptr, error);
    const std::string filtered_path = filtered_config(config_path);
    void* filtered_context = loaded.api->create_context(filtered_path.c_str(), error, sizeof(error));
    require(filtered_context != nullptr, error);

    auto* plain_rate = backend<adapter::IRateControlBackend>(loaded, plain_context, plugin::BackendKind::RateControl);
    auto* filtered_rate = backend<adapter::IRateControlBackend>(loaded, filtered_context, plugin::BackendKind::RateControl);
    adapter::RateControlInput rate{};
    rate.dt_sec = 1.0 / 400.0;
    double plain_energy = 0.0, filtered_energy = 0.0;
    for (int i = 0; i < 200; ++i) {
        rate.rate.p = (i & 1) ? 1.0 : -1.0;
        plain_energy += std::abs(plain_rate->run(rate).x);
        filtered_energy += std::abs(filtered_rate->run(rate).x);
    }
    require(filtered_energy < plain_energy * 0.5, "gyro filter attenuates rate input");

    auto* plain_alt = backend<adapter::IAltitudeControlBackend>(loaded, plain_context, plugin::BackendKind::AltitudeControl);
    auto* filtered_alt = backend<adapter::IAltitudeControlBackend>(loaded, filtered_context, plugin::BackendKind::AltitudeControl);
    adapter::AltitudeControlInput altitude{};
    altitude.mode = adapter::AltitudeControlMode::Velocity;
    double plain_delta = 0.0, filtered_delta = 0.0;
    for (int i = 0; i < 100; ++i) {
        altitude.acceleration.az = (i & 1) ? 10.0 : -10.0;
        plain_delta += std::abs(plain_alt->run(altitude, 0.02).body_z + 1.0);
        filtered_delta += std::abs(filtered_alt->run(altitude, 0.02).body_z + 1.0);
    }
    require(filtered_delta < plain_delta, "accelerometer filter attenuates altitude input");

    auto* plain_pos = backend<adapter::IPositionControl3DBackend>(loaded, plain_context, plugin::BackendKind::PositionControl3D);
    auto* filtered_pos = backend<adapter::IPositionControl3DBackend>(loaded, filtered_context, plugin::BackendKind::PositionControl3D);
    adapter::PositionControl3DVelocityInput position{};
    double plain_thrust = 0.0, filtered_thrust = 0.0;
    for (int i = 0; i < 100; ++i) {
        position.state.acceleration.z = (i & 1) ? 10.0 : -10.0;
        plain_thrust += std::abs(plain_pos->run_velocity(position, 0.02).thrust.body_z);
        filtered_thrust += std::abs(filtered_pos->run_velocity(position, 0.02).thrust.body_z);
    }
    require(filtered_thrust < plain_thrust, "accelerometer filter attenuates 3D position input");

    loaded.api->destroy_backend(plugin::BackendKind::RateControl, plain_rate);
    loaded.api->destroy_backend(plugin::BackendKind::RateControl, filtered_rate);
    loaded.api->destroy_backend(plugin::BackendKind::AltitudeControl, plain_alt);
    loaded.api->destroy_backend(plugin::BackendKind::AltitudeControl, filtered_alt);
    loaded.api->destroy_backend(plugin::BackendKind::PositionControl3D, plain_pos);
    loaded.api->destroy_backend(plugin::BackendKind::PositionControl3D, filtered_pos);
    loaded.api->destroy_context(plain_context);
    loaded.api->destroy_context(filtered_context);
}

} // namespace

int main(int argc, char** argv)
{
    require(argc == 3, "usage: test plugin config");
    biquad_matches_ardupilot_response();
    loader_defaults_filters_to_zero(argv[2]);
    plugin_filters_reduce_high_frequency_inputs(argv[1], argv[2]);
    std::cout << "ap_sensor_filters_smoke: PASS\n";
}
