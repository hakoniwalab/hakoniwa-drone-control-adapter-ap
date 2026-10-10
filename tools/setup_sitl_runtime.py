#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Make an ArduPilot SITL runtime from a Drone PRO PID tuning profile.

The ArduPilot counterpart of Drone PRO's tools/setup_px4_sitl_runtime.py.
From a completed tuning profile (its runtime/all) it writes a runtime
directory with:

  params.parm        the tuning result as ArduPilot parameters
                     (tuning/export_ardupilot_params.py)
  sitl.parm          SITL-only parameters: no pre-arm checks (the SITL has no
                     calibration and its main loop runs at the sensor period),
                     SIM_SPEEDUP 1 (DataFlash logs work), SITL GPS at the rate the tuning assumed,
                     INS_ACCEL_FILTER scaled so the SITL's accel filter has the tuned cutoff
                     (see sitl_accel_filter_hz), and the vehicle's sensor delays
                     (components.sensors.gps/baro.delayMsec) as SIM_GPS_LAG_MS / SIM_BARO_DELAY:
                     ArduPilot SITL makes its own GPS and barometer from the vehicle's state, so the
                     delay the tuning ran with and the EKF allows for (params.parm) is applied there
  drone_config/      the vehicle for Drone PRO's aircraft_service_ardupilot:
                     the profile's 3 ms vehicle with the SITL section
                     (actuator_to_rotor_index) of --sitl-drone-config
  run-service.bash   Drone PRO's standalone aircraft service (terminal 1)
  run-arducopter.bash arducopter with the two parameter files (terminal 2)
  ardupilot-sitl-runtime.json  what the runtime was made from

Usage:
  python tools/setup_sitl_runtime.py \\
    --pro-root ../hakoniwa-drone-pro \\
    --profile ../hakoniwa-drone-pro/work/pid-tuning/<profile> \\
    --sitl-drone-config ../hakoniwa-drone-pro/tuning/vehicle/eams/config/ardupilot-sitl \\
    --out work/sitl/<name>

Build arducopter first: bash tools/build-sitl.bash
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXPORTER = ROOT / "tuning" / "export_ardupilot_params.py"
ARDUCOPTER = ROOT / "thirdparty" / "ardupilot" / "build-hako-sitl" / "sitl" / "bin" / "arducopter"
MARKER = "ardupilot-sitl-runtime.json"
SITL_PORT_SENSOR_IN = 9003   # ArduPilot <- Drone PRO sensors
SITL_PORT_ACTUATOR_OUT = 9002  # ArduPilot -> Drone PRO actuators
MAVLINK_PORT = 14550
# ArduPilot SITL declares its IMU at this rate (AP_InertialSensor_SITL.h, INS_SITL_SENSOR_A) and builds the
# accel filter for it.
SITL_DECLARED_ACCEL_HZ = 1000.0
ARDUPILOT_DEFAULT_ACCEL_FILTER_HZ = 10.0


def read_parm(path: Path, name: str) -> float | None:
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split("#", 1)[0].replace(",", " ").split()
        if len(fields) >= 2 and fields[0] == name:
            return float(fields[1])
    return None


def sensor_delay_msec(drone_config: dict, sensor: str) -> float:
    """components.sensors.<sensor>.delayMsec of a Drone PRO vehicle (0 when absent)."""
    return float(drone_config.get("components", {}).get("sensors", {}).get(sensor, {}).get("delayMsec", 0.0))


def sitl_accel_filter_hz(tuned_cutoff_hz: float, sim_step_sec: float) -> float:
    """INS_ACCEL_FILTER for the SITL so its accel filter has the tuned cutoff.

    In lockstep the SITL makes one IMU sample per simulation step (3 ms: 333 Hz), but ArduPilot builds
    the accel low-pass for the declared 1000 Hz and, unlike the gyro filter, never rebuilds it for the
    observed rate (AP_InertialSensor_Backend::update_accel_filters). Its cutoff in real time is then
    INS_ACCEL_FILTER x 333/1000: 10 Hz acts as 3.3 Hz, about 45 ms more lag at the altitude loop's
    frequency than the vehicle (and the tuning) has, and the tuned vertical loop oscillates in SITL.
    Scaling by the same ratio gives the tuned cutoff. A flight controller samples its IMU at the declared
    rate, so the vehicle keeps the tuned value (params.parm); this is for the SITL only.
    """
    samples_per_sec = 1.0 / sim_step_sec
    if samples_per_sec >= SITL_DECLARED_ACCEL_HZ:
        return tuned_cutoff_hz
    return tuned_cutoff_hz * SITL_DECLARED_ACCEL_HZ / samples_per_sec


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Make an ArduPilot SITL runtime from a PID tuning profile.")
    parser.add_argument("--pro-root", required=True)
    parser.add_argument("--profile", required=True, help="tuning profile directory (with runtime/all)")
    parser.add_argument("--sitl-drone-config", required=True,
                        help="directory with the vehicle's SITL drone_config_0.json (simulation.sitl) and MuJoCo model")
    parser.add_argument("--out", required=True)
    parser.add_argument("--gps-hz", type=float, default=5.0, help="SIM_GPS_HZ (default 5, ArduPilot's default)")
    parser.add_argument("--service", help="Drone PRO's aircraft_service_ardupilot binary "
                        "(default <pro-root>/.hako/build/main_for_sample/service/aircraft_service_ardupilot)")
    args = parser.parse_args()

    pro_root = Path(args.pro_root).resolve()
    profile = Path(args.profile).resolve()
    runtime = profile / "runtime" / "all"
    if not (runtime / "drone_config" / "drone_config_0.json").is_file():
        print(f"ERROR: the profile has no runtime/all (run the ALL manifest first): {runtime}", file=sys.stderr)
        return 1
    sitl_source = Path(args.sitl_drone_config).resolve()
    sitl_config = load_json(sitl_source / "drone_config_0.json")
    if "sitl" not in sitl_config.get("simulation", {}):
        print(f"ERROR: {sitl_source}/drone_config_0.json has no simulation.sitl (actuator_to_rotor_index)", file=sys.stderr)
        return 1
    out = Path(args.out).resolve()
    if out.exists() and not (out / MARKER).is_file():
        print(f"ERROR: {out} exists and is not a runtime made by this tool", file=sys.stderr)
        return 1
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)

    # 1. Parameters from the tuning
    params = out / "params.parm"
    subprocess.run([sys.executable, str(EXPORTER), "--pro-root", str(pro_root), "--runtime", str(runtime),
                    "--out", str(params)], check=True)

    # 2. SITL-only parameters
    tuned = load_json(runtime / "drone_config" / "drone_config_0.json")
    sim_step_sec = float(tuned["simulation"]["timeStep"])
    tuned_accel_filter_hz = read_parm(params, "INS_ACCEL_FILTER")
    if tuned_accel_filter_hz is None:
        tuned_accel_filter_hz = ARDUPILOT_DEFAULT_ACCEL_FILTER_HZ
    (out / "sitl.parm").write_text(
        "# SITL only: not part of the tuning result\n"
        "ARMING_CHECK,0\n"          # no accel calibration in SITL; main loop at the sensor period (< 400 Hz)
        # SIM_SPEEDUP must be set for the AirSim backend: AP_Logger_File scales its IO timeout by it
        # (AP_Logger_File.cpp, timeout_ms *= sitl->speedup) and an unset value trapped the float
        # conversion, which aborted SITL with file logging on.
        "SIM_SPEEDUP,1\n"
        f"SIM_GPS_HZ,{args.gps_hz:g}\n"
        f"# accel filter: {tuned_accel_filter_hz:g} Hz on the vehicle; the SITL's {1.0 / sim_step_sec:.0f} Hz IMU needs\n"
        f"# this value to get it (tools/setup_sitl_runtime.py, sitl_accel_filter_hz)\n"
        f"INS_ACCEL_FILTER,{sitl_accel_filter_hz(tuned_accel_filter_hz, sim_step_sec):g}\n"
        "# sensor delays of the vehicle (drone_config components.sensors.*.delayMsec)\n"
        f"SIM_GPS_LAG_MS,{sensor_delay_msec(tuned, 'gps'):g}\n"
        f"SIM_BARO_DELAY,{sensor_delay_msec(tuned, 'baro'):g}\n",
        encoding="utf-8")

    # 3. The vehicle for the aircraft service: the tuning's 3 ms vehicle, the SITL section of the reference
    tuned_model = runtime / "drone_config" / Path(tuned["components"]["droneDynamics"]["mujoco"]["modelPath"]).name
    if not tuned_model.is_file():
        tuned_model = (pro_root / tuned["components"]["droneDynamics"]["mujoco"]["modelPath"]).resolve()
    config_dir = out / "drone_config"
    config_dir.mkdir()
    shutil.copy2(tuned_model, config_dir / "drone.xml")
    tuned["components"]["droneDynamics"]["mujoco"]["modelPath"] = str(config_dir / "drone.xml")
    tuned["simulation"]["sitl"] = sitl_config["simulation"]["sitl"]
    # Sensors to ArduPilot every physics step (the AirSim backend expects one sensor frame per servo frame)
    tuned["simulation"].setdefault("mavlink_tx_period_msec", {})["hil_sensor"] = max(1, int(round(sim_step_sec * 1000)))
    for key in ("controller",):
        tuned[key] = sitl_config.get(key, tuned.get(key))  # ArduPilot flies; the service's controller is the SITL one
    write_json(config_dir / "drone_config_0.json", tuned)

    # 4. Launch scripts
    service = Path(args.service).resolve() if args.service else \
        pro_root / ".hako" / "build" / "main_for_sample" / "service" / "aircraft_service_ardupilot"
    if not service.is_file():
        print(f"ERROR: the aircraft service was not found: {service} (build Drone PRO, or pass --service)", file=sys.stderr)
        return 1
    (out / "run-service.bash").write_text(f"""#!/usr/bin/env bash
# Terminal 1: Drone PRO's standalone aircraft service (MuJoCo, 3 ms), paced to real time.
cd {pro_root}
DYLD_LIBRARY_PATH=vendor/mujoco/lib \\
  {service} \\
  127.0.0.1 {SITL_PORT_ACTUATOR_OUT} {SITL_PORT_SENSOR_IN} \\
  {config_dir} \\
  --real-sleep-msec {int(round(float(tuned['simulation']['timeStep']) * 1000))}
""", encoding="utf-8")
    (out / "run-arducopter.bash").write_text(f"""#!/usr/bin/env bash
# Terminal 2: ArduCopter SITL with the tuning result. Add --wipe on the first start after a frame change.
cd {out}
{ARDUCOPTER} \\
  --model airsim-copter \\
  --sim-address=127.0.0.1 --sim-port-in {SITL_PORT_SENSOR_IN} --sim-port-out {SITL_PORT_ACTUATOR_OUT} \\
  --serial0=udpclient:127.0.0.1:{MAVLINK_PORT} \\
  --defaults {params},{out / 'sitl.parm'} "$@"
""", encoding="utf-8")
    for script in ("run-service.bash", "run-arducopter.bash"):
        (out / script).chmod(0o755)

    write_json(out / MARKER, {
        "profile": str(profile), "runtime": str(runtime), "sitl_drone_config": str(sitl_source),
        "arducopter": str(ARDUCOPTER), "arducopter_built": ARDUCOPTER.is_file(),
    })
    print(f"OK: {out}")
    if not ARDUCOPTER.is_file():
        print(f"NOTE: arducopter is not built yet: bash tools/build-sitl.bash", file=sys.stderr)
    print(f"  terminal 1: bash {out}/run-service.bash")
    print(f"  terminal 2: bash {out}/run-arducopter.bash")
    print(f"  fly:        python {pro_root}/drone_api/mavlink_rpc/commands/takeoff_client.py 5 --connection udpin:127.0.0.1:{MAVLINK_PORT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
