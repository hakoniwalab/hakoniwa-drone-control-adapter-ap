#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Prepare a vehicle for ArduPilot PID tuning under ArduPilot SITL conditions.

The ArduPilot counterpart of Drone PRO's tuning/px4/tools/prepare_px4_inputs.py. From the
customer's Hakoniwa vehicle (controller-params.txt, drone.xml, drone_config_0.json) it writes the
inputs of a Drone PRO `plugin` tuning profile:

  controller-params.txt, drone.xml   prepared by prepare_px4_inputs.py (shared X/Y gains made
                                     equal, GNSS quality defaults); control cycles come from the
                                     timing profile (tuning/sitl-1ms-timing.json: 1 ms physics, 3 ms control) at profile creation
  drone_config_0.json                controller.backendType adapter-plugin with this repository's
                                     plugin, the EKF in the loop at the SITL sensor periods
  ardupilot-controller-config.json   config/ardupilot-controller-config.json with the SITL conditions:
                                     333 Hz stages, ArduPilot's IMU filters and rate filters, and
                                     MOT_THST_HOVER from the rotor model (tuning/ardupilot_vehicle.py,
                                     the same computation the export uses), and the sensor delays the
                                     EKF allows for (GPS1_DELAY_MS, EK3_HGT_DELAY) equal to the vehicle's

Sensor delays (Drone PRO components.sensors.gps/baro.delayMsec, applied by a runner built with
HAKO_SENSOR_DELAY): the vehicle's value when it has one, else ArduPilot SITL's (GPS SIM_GPS_LAG_MS
100 ms; barometer 60 ms, the EK3_HGT_DELAY the plugin's NavEKF3 keeps). The tuning, the export and the
SITL runtime all take them from drone_config_0.json, so the EKF's allowance matches the delay.

Usage:
  python tuning/prepare_ardupilot_inputs.py --pro-root ../hakoniwa-drone-pro \\
    ../hakoniwa-drone-pro/work/my-drone-config ../hakoniwa-drone-pro/work/my-drone-config-ap
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ardupilot_vehicle import motor_thrust_model  # noqa: E402

TEMPLATE = ROOT / "config" / "ardupilot-controller-config.json"
PLUGIN = ROOT / "install" / "lib" / "libhakoniwa_drone_control_adapter_ap_plugin.so"
SITL_RATE_HZ = 1.0 / 0.003  # ArduCopter's main loop in SITL: every Hakoniwa physics step
# EKF sensor periods as ArduPilot SITL sees them: IMU each step, compass 50 Hz, baro 100 Hz,
# GPS 5 Hz (SIM_GPS_HZ default)
EKF_INTERVALS_USEC = {"imuIntervalUsec": 3000, "magIntervalUsec": 20000,
                      "baroIntervalUsec": 10000, "gpsIntervalUsec": 200000}
# ArduPilot's defaults the SITL runs with (AP_InertialSensor INS_*_FILTER, AC_AttitudeControl_Multi
# ATC_RAT_*_FLTT/FLTD as the EAMS SITL parameter file, AC_AttitudeControl ATC_ACCEL_*_MAX)
SITL_CONDITIONS = {
    "INS_GYRO_FILTER": 20, "INS_ACCEL_FILTER": 10,
    "ATC_RAT_RLL_FLTT": 10, "ATC_RAT_RLL_FLTD": 10,
    "ATC_RAT_PIT_FLTT": 10, "ATC_RAT_PIT_FLTD": 10,
    "ATC_ACCEL_R_MAX": 110000, "ATC_ACCEL_P_MAX": 110000, "ATC_ACCEL_Y_MAX": 27000,
}
# The interface contract test turns these off in the template; tuning runs with ArduPilot's dynamics.
CONTRACT_TEST_ONLY_RUNTIME = ("throttle_filter_hz", "position_input_shaping")
CONTRACT_TEST_ONLY_PARAMETERS = ("ATC_RATE_FF_ENAB",)
# ArduPilot SITL's sensor delays (SIM_GPS_LAG_MS default; EK3_HGT_DELAY default, which the plugin's
# NavEKF3 cannot change), used when the vehicle sets none
DEFAULT_SENSOR_DELAY_MSEC = {"gps": 100.0, "baro": 60.0}


def parse_param_text(path: Path) -> dict[str, float]:
    params: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split("#", 1)[0].split()
        if len(fields) >= 2:
            try:
                params[fields[0]] = float(fields[1])
            except ValueError:
                pass
    return params


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a vehicle for ArduPilot PID tuning (SITL conditions).")
    parser.add_argument("--pro-root", required=True, help="hakoniwa-drone-pro checkout")
    parser.add_argument("source_dir", type=Path, help="the customer's Hakoniwa vehicle directory")
    parser.add_argument("output_dir", type=Path, help="new directory for the ArduPilot tuning inputs")
    args = parser.parse_args()

    pro_root = Path(args.pro_root).resolve()
    output = args.output_dir.resolve()
    # 1. The PX4 preparation: shared X/Y gains, GNSS quality defaults, the drone.xml copy.
    subprocess.run([sys.executable, str(pro_root / "tuning/px4/tools/prepare_px4_inputs.py"),
                    str(args.source_dir.resolve()), str(output)], check=True, cwd=pro_root)
    sys.path.insert(0, str(pro_root))
    from tuning.px4.conversion.native_params import compute_hover_thrust_fraction  # type: ignore

    changes: list[str] = []
    drone_path = output / "drone_config_0.json"
    drone = json.loads(drone_path.read_text(encoding="utf-8"))
    controller = drone.setdefault("controller", {})
    controller["backendType"] = "adapter-plugin"
    controller["backendConfig"] = {"pluginPath": str(PLUGIN), "configPath": "ardupilot-controller-config.json"}
    for key in ("px4ConfigPath",):
        controller.get("backendConfig", {}).pop(key, None)
    # Altitude above the takeoff point, as ArduPilot holds it above home: a target of 5 m in the
    # tuning is the same height as a 5 m takeoff in SITL (the world frame adds the vehicle's rest height).
    controller["ekf"] = {"enable": True, **EKF_INTERVALS_USEC, "altitudeReference": "takeoff"}
    changes.append("controller: adapter-plugin (ArduPilot), EKF at the SITL sensor periods, altitude from the takeoff point")
    sensors = drone.setdefault("components", {}).setdefault("sensors", {})
    delays: dict[str, float] = {}
    for name, default in DEFAULT_SENSOR_DELAY_MSEC.items():
        sensor = sensors.setdefault(name, {})
        if "delayMsec" not in sensor:
            sensor["delayMsec"] = default
            changes.append(f"sensors.{name}.delayMsec {default:g} (ArduPilot SITL)")
        delays[name] = float(sensor["delayMsec"])
    drone_path.write_text(json.dumps(drone, indent=2) + "\n", encoding="utf-8")

    config = json.loads(TEMPLATE.read_text(encoding="utf-8"))
    for key in CONTRACT_TEST_ONLY_RUNTIME:
        config.get("runtime", {}).pop(key, None)
    for key in CONTRACT_TEST_ONLY_PARAMETERS:
        config["parameters"].pop(key, None)
    for key in config["runtime"]:
        if key.endswith("_hz"):
            config["runtime"][key] = SITL_RATE_HZ
    config["parameters"].update(SITL_CONDITIONS)
    controller_params = parse_param_text(output / "controller-params.txt")
    model = motor_thrust_model(drone, controller_params, compute_hover_thrust_fraction)
    config["parameters"]["MOT_THST_HOVER"] = model["thrust_hover"]
    config["parameters"]["GPS1_DELAY_MS"] = delays["gps"]
    config["parameters"]["EK3_HGT_DELAY"] = delays["baro"]
    changes.append(f"EKF allowance: GPS1_DELAY_MS {delays['gps']:g}, EK3_HGT_DELAY {delays['baro']:g} (the sensor delays)")
    changes.append(f"runtime: every stage {SITL_RATE_HZ:.2f} Hz (ArduCopter's main loop in SITL)")
    changes.append("parameters: " + ", ".join(f"{k} {v:g}" for k, v in SITL_CONDITIONS.items()))
    changes.append(f"MOT_THST_HOVER {model['thrust_hover']:.4f} from the rotor model (hover duty {model['hover_duty']:.3f})")
    (output / "ardupilot-controller-config.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")

    print(f"Prepared ArduPilot tuning inputs: {output}")
    for change in changes:
        print(f"  {change}")
    if not PLUGIN.is_file():
        print(f"NOTE: the plugin is not installed yet: bash build.bash install ({PLUGIN})", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
