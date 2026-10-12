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
import math
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from ardupilot_vehicle import motor_thrust_model  # noqa: E402
sys.path.insert(0, str(ROOT / "tools"))
from ardupilot_declination import magnetic_field  # noqa: E402

TEMPLATE = ROOT / "config" / "ardupilot-controller-config.json"
PLUGIN = ROOT / "install" / "lib" / "libhakoniwa_drone_control_adapter_ap_plugin.so"
SITL_RATE_HZ = 1.0 / 0.003  # ArduCopter's main loop in SITL: every Hakoniwa physics step
# EKF sensor periods as ArduPilot SITL sees them: IMU each step, compass 10 Hz (ArduCopter reads the
# compass in a 10 Hz task, ArduCopter/Copter.cpp; at 50 Hz the plugin's yaw was held harder by the
# compass than SITL's, x500 2026-10-12), baro 100 Hz, GPS 5 Hz (SIM_GPS_HZ default)
EKF_INTERVALS_USEC = {"imuIntervalUsec": 3000, "magIntervalUsec": 100000,
                      "baroIntervalUsec": 10000, "gpsIntervalUsec": 200000}
# Mission Planner plugins/InitialParamsCalculator.cs calc_values() is the source of these initial
# values. They replace the lower ArduPilot defaults because the x500 hovered with a persistent
# approximately 5.6 Hz, +/-1.4 degree oscillation at 20/10 Hz, but not at the calculator's 42/21 Hz.
INITIAL_PARAMETER_SOURCE = "MissionPlanner plugins/InitialParamsCalculator.cs calc_values()"
# The interface contract test turns these off in the template; tuning runs with ArduPilot's dynamics.
CONTRACT_TEST_ONLY_RUNTIME = ("throttle_filter_hz", "position_input_shaping")
CONTRACT_TEST_ONLY_PARAMETERS = ("ATC_RATE_FF_ENAB",)
# ArduPilot SITL's sensor delays (SIM_GPS_LAG_MS default; EK3_HGT_DELAY default, which the plugin's
# NavEKF3 cannot change), used when the vehicle sets none
DEFAULT_SENSOR_DELAY_MSEC = {"gps": 100.0, "baro": 60.0}
# Match the standard deviation of ArduPilot SITL's uniform SIM_BARO_RND +/-0.2 m.
# Drone PRO's SensorNoise is Gaussian; matching the distribution shape is intentionally out of scope.
SITL_MAG_NOISE_GAUSS = 0.0
SITL_BARO_NOISE_ALT_STD_M = 0.2 / math.sqrt(3.0)


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


def propeller_inches_from_mujoco(path: Path) -> int:
    """Return the Mission Planner prop-size input (whole inches) from propeller cylinder radii."""
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"cannot read MuJoCo model {path}: {exc}") from exc
    radii: list[float] = []
    for geom in root.iter("geom"):
        name = geom.get("name", "").lower()
        if geom.get("type") != "cylinder" or "prop" not in name or "motor" in name:
            continue
        size = geom.get("size", "").split()
        try:
            radius = float(size[0])
        except (IndexError, ValueError):
            continue
        if radius > 0:
            radii.append(radius)
    if not radii:
        raise ValueError(f"no named propeller cylinder radius found in {path}; use --prop-inch")
    if not math.isclose(min(radii), max(radii), rel_tol=1e-6, abs_tol=1e-9):
        raise ValueError(f"propeller cylinder radii are inconsistent in {path}; use --prop-inch")
    # Mission Planner's prop-size control accepts whole inches. Match that input by rounding the
    # MuJoCo diameter (metres) to the nearest inch rather than feeding a fractional model value.
    prop_inch = round(2.0 * radii[0] / 0.0254)
    if prop_inch <= 0:
        raise ValueError(f"propeller diameter in {path} rounds to zero inches; use --prop-inch")
    return prop_inch


def initial_parameters(prop_inch: int) -> tuple[dict[str, int | float], dict[str, str]]:
    if prop_inch <= 0:
        raise ValueError("propeller diameter must be positive")
    gyro = max(20, round(289.22 * prop_inch ** -0.838))
    rate_filter = max(10, gyro / 2)
    accel_rp = max(10000, round(
        (-2.613267 * prop_inch ** 3 + 343.39216 * prop_inch ** 2
         - 15083.7121 * prop_inch + 235771) / 100
    ) * 100)
    accel_y = max(8000, round((-900 * prop_inch + 36000) / 100) * 100)
    values = {
        "INS_GYRO_FILTER": gyro, "INS_ACCEL_FILTER": 20,
        "ATC_RAT_RLL_FLTT": rate_filter, "ATC_RAT_RLL_FLTD": rate_filter,
        "ATC_RAT_RLL_FLTE": 0,
        "ATC_RAT_PIT_FLTT": rate_filter, "ATC_RAT_PIT_FLTD": rate_filter,
        "ATC_RAT_PIT_FLTE": 0,
        "ATC_RAT_YAW_FLTT": rate_filter, "ATC_RAT_YAW_FLTD": 0,
        "ATC_RAT_YAW_FLTE": 2,
        "ATC_ACCEL_R_MAX": accel_rp, "ATC_ACCEL_P_MAX": accel_rp,
        "ATC_ACCEL_Y_MAX": accel_y,
    }
    formulas = {
        "INS_GYRO_FILTER": "max(20, round(289.22 * prop_inch^-0.838))",
        "roll/pitch FLTD,FLTT; yaw FLTT": "max(10, INS_GYRO_FILTER / 2)",
        "roll/pitch FLTE; yaw FLTD,FLTE": "0; 0,2",
        "ATC_ACCEL_R/P_MAX": "max(10000, round((-2.613267*p^3 + 343.39216*p^2 - 15083.7121*p + 235771) / 100) * 100)",
        "ATC_ACCEL_Y_MAX": "max(8000, round((-900*p + 36000) / 100) * 100)",
        "INS_ACCEL_FILTER": "20",
    }
    return values, formulas


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare a vehicle for ArduPilot PID tuning (SITL conditions).")
    parser.add_argument("--pro-root", required=True, help="hakoniwa-drone-pro checkout")
    parser.add_argument("--prop-inch", type=int, help="propeller diameter in whole inches (instead of drone.xml)")
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
    location = drone["simulation"]["location"]
    field = magnetic_field(float(location["latitude"]), float(location["longitude"]))
    location["magneticField"] = {
        "intensity_nT": field["intensity_gauss"] * 100000.0,
        "declination_deg": field["declination_deg"],
        "inclination_deg": field["inclination_deg"],
    }
    # AP SITL starts its triangular bias clock at autopilot startup; the prepared Control Link run
    # starts Hakoniwa's clock at the same simulation epoch.
    sensors.setdefault("gyro", {})["drift"] = {
        "mode": "ardupilot-sitl", "speedDegPerSecPerMin": 0.05, "timeMin": 5.0
    }
    changes.append(
        "magnetic field: AP_Declination table at home "
        f"(declination {field['declination_deg']:.6f} deg, inclination {field['inclination_deg']:.6f} deg, "
        f"intensity {field['intensity_gauss']:.6f} gauss)"
    )
    changes.append("gyro drift: ArduPilot SITL default 0.05 deg/s/min triangular over 5 min")
    sensors.setdefault("mag", {})["noise"] = SITL_MAG_NOISE_GAUSS
    sensors.setdefault("baro", {})["noise"] = SITL_BARO_NOISE_ALT_STD_M
    drone.setdefault("notes", []).extend([
        "mag noise 0 gauss matches ArduPilot SITL SIM_MAG_RND=0",
        "baro noise std 0.2/sqrt(3) m matches the standard deviation of ArduPilot SITL "
        "SIM_BARO_RND=0.2 uniform +/-0.2 m; Drone PRO noise remains Gaussian",
        "the ArduPilot adapter EKF uses pressure_alt (m), not abs_pressure (Pa); Drone PRO also adds "
        "the same numeric std to abs_pressure, so its incidental pressure noise is 0.115 Pa "
        "(about 0.01 m altitude near sea level)",
    ])
    changes.append("mag noise: 0 gauss (SIM_MAG_RND 0)")
    changes.append(
        f"baro noise: {SITL_BARO_NOISE_ALT_STD_M:.9g} m Gaussian std "
        "(same std as SIM_BARO_RND 0.2 uniform +/-0.2 m)"
    )
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
    try:
        prop_inch = (args.prop_inch if args.prop_inch is not None
                     else propeller_inches_from_mujoco(output / "drone.xml"))
        initial, formulas = initial_parameters(prop_inch)
    except ValueError as exc:
        parser.error(str(exc))
    config["parameters"].update(initial)
    config["initial_parameters"] = {
        "source": INITIAL_PARAMETER_SOURCE,
        "propeller_diameter_inch": prop_inch,
        "formulas": formulas,
        "reason": "x500 SITL hover oscillated at the old 20/10 Hz filters; Mission Planner's 42/21 Hz was stable",
    }
    controller_params = parse_param_text(output / "controller-params.txt")
    model = motor_thrust_model(drone, controller_params, compute_hover_thrust_fraction)
    config["parameters"]["MOT_THST_HOVER"] = model["thrust_hover"]
    config["parameters"]["GPS1_DELAY_MS"] = delays["gps"]
    config["parameters"]["EK3_HGT_DELAY"] = delays["baro"]
    changes.append(f"EKF allowance: GPS1_DELAY_MS {delays['gps']:g}, EK3_HGT_DELAY {delays['baro']:g} (the sensor delays)")
    changes.append(f"runtime: every stage {SITL_RATE_HZ:.2f} Hz (ArduCopter's main loop in SITL)")
    changes.append(f"initial parameters: {INITIAL_PARAMETER_SOURCE}, propeller {prop_inch} inch")
    changes.append("parameters: " + ", ".join(f"{k} {v:g}" for k, v in initial.items()))
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
