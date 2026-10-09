#!/usr/bin/env python3
"""Export a Drone PRO PID tuning result as ArduPilot parameters (a .parm file).

The ArduPilot counterpart of Drone PRO's tuning/px4/tools/export_px4_params.py:
the tuned PID_* parameters, the adapter configuration the tuning ran with
(filters, limits) and the values derived from the vehicle (hover thrust,
thrust curve, frame) become the parameters ArduPilot SITL loads with
--defaults. Parameter names are those of the ArduPilot checkout in
thirdparty/ardupilot (ArduCopter/Parameters.cpp, AC_PosControl, AC_WPNav).

Usage:
  python tuning/export_ardupilot_params.py \\
    --pro-root ../hakoniwa-drone-pro \\
    --runtime ../hakoniwa-drone-pro/work/pid-tuning/<profile>/runtime/all \\
    --out <profile>.parm

--runtime is the runtime directory a completed pipeline writes
(drone_config/drone_config_0.json, the controller parameters it names and the
adapter configuration its controller.backendConfig.configPath names).
The vehicle-derived values (MOT_THST_HOVER, MOT_THST_EXPO) use Drone PRO's
rotor model derivation (tuning/px4/conversion/native_params.py) at the hover
point, through ArduPilot's output chain (MOT_SPIN_MIN/MAX, MOT_PWM_MIN/MAX)
and the SITL service's PWM-to-duty reading.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

# Hakoniwa PID_* -> ArduPilot (ArduCopter/Parameters.cpp: ATC_ = AC_AttitudeControl,
# PSC = AC_PosControl; AC_PosControl.cpp: _POSZ_, _VELZ_, _ACCZ_, _POSXY_, _VELXY_).
PID_TO_ARDUPILOT = {
    "PID_ROLL_RATE_Kp": "ATC_RAT_RLL_P",
    "PID_ROLL_RATE_Ki": "ATC_RAT_RLL_I",
    "PID_ROLL_RATE_Kd": "ATC_RAT_RLL_D",
    "PID_PITCH_RATE_Kp": "ATC_RAT_PIT_P",
    "PID_PITCH_RATE_Ki": "ATC_RAT_PIT_I",
    "PID_PITCH_RATE_Kd": "ATC_RAT_PIT_D",
    "PID_YAW_RATE_Kp": "ATC_RAT_YAW_P",
    "PID_YAW_RATE_Ki": "ATC_RAT_YAW_I",
    "PID_YAW_RATE_Kd": "ATC_RAT_YAW_D",
    "PID_ROLL_Kp": "ATC_ANG_RLL_P",
    "PID_PITCH_Kp": "ATC_ANG_PIT_P",
    "PID_YAW_Kp": "ATC_ANG_YAW_P",
    "PID_ALT_Kp": "PSC_POSZ_P",
    "PID_ALT_SPD_Kp": "PSC_VELZ_P",
    "PID_ALT_SPD_Ki": "PSC_VELZ_I",
    "PID_ALT_SPD_Kd": "PSC_VELZ_D",
}
# X and Y share one ArduPilot gain; the tuning keeps them equal.
SHARED_AXIS_PID_TO_ARDUPILOT = {
    ("PID_POS_X_Kp", "PID_POS_Y_Kp"): "PSC_POSXY_P",
    ("PID_POS_VX_Kp", "PID_POS_VY_Kp"): "PSC_VELXY_P",
    ("PID_POS_VX_Ki", "PID_POS_VY_Ki"): "PSC_VELXY_I",
    ("PID_POS_VX_Kd", "PID_POS_VY_Kd"): "PSC_VELXY_D",
}
# Adapter configuration keys exported as they are (the adapter loads them in
# ArduPilot's own units) when the configuration has them.
ADAPTER_CONFIG_PASSTHROUGH = (
    "INS_GYRO_FILTER", "INS_ACCEL_FILTER",
    "ATC_RAT_RLL_FLTT", "ATC_RAT_RLL_FLTE", "ATC_RAT_RLL_FLTD", "ATC_RAT_RLL_IMAX", "ATC_RAT_RLL_FF",
    "ATC_RAT_PIT_FLTT", "ATC_RAT_PIT_FLTE", "ATC_RAT_PIT_FLTD", "ATC_RAT_PIT_IMAX", "ATC_RAT_PIT_FF",
    "ATC_RAT_YAW_FLTT", "ATC_RAT_YAW_FLTE", "ATC_RAT_YAW_FLTD", "ATC_RAT_YAW_IMAX", "ATC_RAT_YAW_FF",
    "PSC_VELZ_IMAX", "PSC_VELZ_FLTE", "PSC_VELZ_FLTD", "PSC_VELZ_FF",
    "PSC_ACCZ_P", "PSC_ACCZ_I", "PSC_ACCZ_D", "PSC_ACCZ_IMAX", "PSC_ACCZ_FLTT", "PSC_ACCZ_FLTE", "PSC_ACCZ_FLTD", "PSC_ACCZ_FF",
    "PSC_VELXY_IMAX", "PSC_VELXY_FLTE", "PSC_VELXY_FLTD", "PSC_VELXY_FF",
    "PSC_JERK_XY", "PSC_JERK_Z",
    "ATC_ACCEL_R_MAX", "ATC_ACCEL_P_MAX", "ATC_ACCEL_Y_MAX", "ATC_INPUT_TC",
)
# The adapter configuration keeps these in SI (m/s^2, thrust 0..1); ArduPilot's are cm/s^2
# (PSC_VEL*_IMAX) and 0.001 thrust (PSC_ACCZ_IMAX, AC_PosControl _pid_accel_z).
ADAPTER_CONFIG_SCALE = {"PSC_VELZ_IMAX": 100.0, "PSC_VELXY_IMAX": 100.0, "PSC_ACCZ_IMAX": 1000.0}
# Frame from the rotor count (AP_MotorsMatrix FRAME_CLASS; FRAME_TYPE 1 = X).
FRAME_CLASS_BY_ROTORS = {4: 1, 6: 2, 8: 3}
RPM_TO_DEG_PER_SEC = 360.0 / 60.0
# ArduPilot defaults (AP_MotorsMulticopter.h); exported so the thrust curve fit holds
SPIN_MIN = 0.15
SPIN_MAX = 0.95


def parse_param_text(path: Path) -> dict[str, float]:
    params: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) >= 2:
            try:
                params[fields[0]] = float(fields[1])
            except ValueError:
                continue
    return params


def require(params: dict[str, float], key: str) -> float:
    if key not in params:
        raise SystemExit(f"ERROR: controller parameter was not found: {key}")
    return params[key]


def require_same(params: dict[str, float], left: str, right: str) -> float:
    a, b = require(params, left), require(params, right)
    if not math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12):
        raise SystemExit(f"ERROR: {left}={a:g} and {right}={b:g} must be equal (ArduPilot has one horizontal gain)")
    return a


def resolve(base: Path, value: str, pro_root: Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    for candidate in (base / path, pro_root / path):
        if candidate.is_file():
            return candidate
    raise SystemExit(f"ERROR: file was not found: {value} (relative to {base} or {pro_root})")


def format_number(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return f"{value:.6g}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Export a PID tuning result as ArduPilot parameters.")
    parser.add_argument("--pro-root", required=True, help="hakoniwa-drone-pro checkout (its tuning tools are used)")
    parser.add_argument("--runtime", required=True, help="the tuning profile's runtime directory (runtime/all)")
    parser.add_argument("--out", required=True, help=".parm to write (NAME,VALUE lines)")
    parser.add_argument("--sitl-period-sec", type=float, default=0.003,
                        help="ArduPilot SITL main loop period, for the cycle check (default 0.003)")
    args = parser.parse_args()

    pro_root = Path(args.pro_root).resolve()
    sys.path.insert(0, str(pro_root))
    from tuning.px4.conversion.native_params import compute_hover_thrust_fraction  # type: ignore

    runtime = Path(args.runtime).resolve()
    drone_config_path = runtime / "drone_config" / "drone_config_0.json"
    if not drone_config_path.is_file():
        raise SystemExit(f"ERROR: runtime drone config was not found: {drone_config_path}")
    drone_config = json.loads(drone_config_path.read_text(encoding="utf-8"))
    controller = drone_config.get("controller", {})
    if controller.get("backendType") != "adapter-plugin":
        raise SystemExit("ERROR: the runtime's controller.backendType must be adapter-plugin")
    controller_params = parse_param_text(resolve(drone_config_path.parent, controller["paramFilePath"], pro_root))
    adapter_config_path = resolve(drone_config_path.parent, controller["backendConfig"]["configPath"], pro_root)
    adapter_config = json.loads(adapter_config_path.read_text(encoding="utf-8"))
    adapter_params = adapter_config.get("parameters", {})

    out: dict[str, float] = {}
    notes: list[str] = []

    # 1. Tuned gains
    for hakoniwa_name, ardupilot_name in PID_TO_ARDUPILOT.items():
        out[ardupilot_name] = require(controller_params, hakoniwa_name)
    for (left, right), ardupilot_name in SHARED_AXIS_PID_TO_ARDUPILOT.items():
        out[ardupilot_name] = require_same(controller_params, left, right)

    # 2. The adapter configuration the tuning ran with (filters, inner loops, limits)
    for name in ADAPTER_CONFIG_PASSTHROUGH:
        if name in adapter_params:
            out[name] = float(adapter_params[name]) * ADAPTER_CONFIG_SCALE.get(name, 1.0)

    # 3. Vehicle limits (Hakoniwa units -> ArduPilot units)
    #    ATC_RATE_*_MAX deg/s, ANGLE_MAX centidegrees, WPNAV_*/LOIT_* cm/s and cm/s^2
    out["ATC_RATE_R_MAX"] = require(controller_params, "PID_ROLL_RPM_MAX") * RPM_TO_DEG_PER_SEC
    out["ATC_RATE_P_MAX"] = require(controller_params, "PID_PITCH_RPM_MAX") * RPM_TO_DEG_PER_SEC
    out["ATC_RATE_Y_MAX"] = require(controller_params, "PID_YAW_RPM_MAX") * RPM_TO_DEG_PER_SEC
    angle_max_deg = min(require(controller_params, "PID_POS_MAX_ROLL"), require(controller_params, "PID_POS_MAX_PITCH"))
    out["ANGLE_MAX"] = angle_max_deg * 100.0
    horizontal_speed = require(controller_params, "PID_POS_MAX_SPD") * 100.0
    out["WPNAV_SPEED"] = horizontal_speed
    out["LOIT_SPEED"] = horizontal_speed
    vertical_speed = require(controller_params, "PID_ALT_MAX_SPD") * 100.0
    out["WPNAV_SPEED_UP"] = vertical_speed
    out["WPNAV_SPEED_DN"] = vertical_speed
    out["PILOT_SPEED_UP"] = vertical_speed
    if "WPNAV_ACCEL_Z" in adapter_params:
        out["WPNAV_ACCEL_Z"] = float(adapter_params["WPNAV_ACCEL_Z"]) * 100.0
    if "PSC_ACCXY_MAX" in adapter_params:
        out["WPNAV_ACCEL"] = float(adapter_params["PSC_ACCXY_MAX"]) * 100.0
        out["LOIT_ACC_MAX"] = out["WPNAV_ACCEL"]

    # 4. Vehicle: frame, hover thrust and thrust curve from the Hakoniwa rotor model.
    #    ArduPilot output chain (AP_MotorsMulticopter): thrust fraction t (0..1 of the
    #    thrust at MOT_SPIN_MAX) -> curve c with c*EXPO^... : t = EXPO*c^2 + (1-EXPO)*c
    #    -> actuator a = SPIN_MIN + (SPIN_MAX-SPIN_MIN)*c -> PWM = PWM_MIN + (PWM_MAX-PWM_MIN)*a.
    #    Drone PRO's SITL service reads duty = (PWM-1000)/1000, so PWM 1000..2000 makes
    #    duty = a. EXPO is solved at the hover point: duty hover_duty (rotor model) must
    #    give thrust MOT_THST_HOVER, as PX4's THR_MDL_FAC (hover point, no least squares).
    rotors = drone_config["components"]["thruster"]["rotorPositions"]
    frame_class = FRAME_CLASS_BY_ROTORS.get(len(rotors))
    if frame_class is None:
        raise SystemExit(f"ERROR: no ArduPilot frame class for {len(rotors)} rotors")
    out["FRAME_CLASS"] = frame_class
    out["FRAME_TYPE"] = 1  # X
    out["MOT_PWM_MIN"] = 1000
    out["MOT_PWM_MAX"] = 2000
    out["MOT_SPIN_MIN"] = SPIN_MIN
    out["MOT_SPIN_MAX"] = SPIN_MAX
    _, hover, _ = compute_hover_thrust_fraction(drone_config, controller_params)
    inputs = hover["inputs"]
    k, r, cq, d, v_bat = inputs["K"], inputs["R"], inputs["Cq"], inputs["D"], inputs["NominalVoltage"]
    omega_max = hover["omega_max_effective"]
    hover_duty = hover["hover_duty"]
    a_coef, b_coef = cq * r / k, k + d * r / k

    def thrust_at_duty(duty: float) -> float:  # relative to the thrust at omega_max
        omega = (-b_coef + math.sqrt(b_coef * b_coef + 4.0 * a_coef * v_bat * duty)) / (2.0 * a_coef) if a_coef > 0 else v_bat * duty / b_coef
        return min(omega, omega_max) ** 2 / omega_max ** 2

    if not SPIN_MIN < hover_duty < SPIN_MAX:
        raise SystemExit(f"ERROR: hover duty {hover_duty:.3f} is outside MOT_SPIN_MIN..MOT_SPIN_MAX ({SPIN_MIN}..{SPIN_MAX})")
    thrust_hover = thrust_at_duty(hover_duty) / thrust_at_duty(SPIN_MAX)  # ArduPilot's fraction of its maximum
    curve_hover = (hover_duty - SPIN_MIN) / (SPIN_MAX - SPIN_MIN)
    raw_expo = (thrust_hover - curve_hover) / (curve_hover * curve_hover - curve_hover)
    expo = min(1.0, max(0.0, raw_expo))
    out["MOT_THST_HOVER"] = thrust_hover
    out["MOT_THST_EXPO"] = expo
    notes.append(
        f"MOT_THST_HOVER={thrust_hover:.4g} (hover duty {hover_duty:.3f}, thrust relative to duty {SPIN_MAX}); "
        f"MOT_THST_EXPO={expo:.4g} fitted at the hover point with MOT_SPIN_MIN/MAX {SPIN_MIN}/{SPIN_MAX}, MOT_PWM 1000..2000"
    )
    if expo != raw_expo:
        notes.append(f"WARNING: MOT_THST_EXPO={raw_expo:.4g} is outside 0..1 and was held to {expo:g}; the hover thrust is off")
    tuned_hover = adapter_params.get("MOT_THST_HOVER")
    if tuned_hover is not None and not math.isclose(float(tuned_hover), thrust_hover, abs_tol=0.02):
        notes.append(f"WARNING: the tuning ran with MOT_THST_HOVER={float(tuned_hover):g}, the vehicle gives {thrust_hover:.4g}")

    # 5. Loop rate from the tuning's cycle; warn when it is not the SITL period
    rate_cycle = controller_params.get("ANGULAR_RATE_CONTROL_CYCLE")
    if rate_cycle:
        out["SCHED_LOOP_RATE"] = round(1.0 / rate_cycle)
        for name in ("ANGULAR_CONTROL_CYCLE", "PID_ALT_CONTROL_CYCLE", "POS_CONTROL_CYCLE", "SPD_CONTROL_CYCLE"):
            value = controller_params.get(name)
            if value is not None and not math.isclose(value, args.sitl_period_sec, rel_tol=1e-6):
                notes.append(f"WARNING: {name}={value:g} s differs from the SITL period {args.sitl_period_sec:g} s "
                             "(ArduCopter runs every controller in its main loop); tune with the SITL timing profile")
        if not math.isclose(rate_cycle, args.sitl_period_sec, rel_tol=1e-6):
            notes.append(f"WARNING: ANGULAR_RATE_CONTROL_CYCLE={rate_cycle:g} s differs from the SITL period {args.sitl_period_sec:g} s")
    step = drone_config.get("simulation", {}).get("timeStep")
    if step is not None and not math.isclose(float(step), args.sitl_period_sec, rel_tol=1e-6):
        notes.append(f"WARNING: simulation.timeStep={step} differs from the SITL period {args.sitl_period_sec:g} s")

    lines = [
        "# Generated by hakoniwa-drone-control-adapter-ap/tuning/export_ardupilot_params.py",
        f"# runtime: {runtime}",
        f"# adapter configuration: {adapter_config_path.name}",
        "# Parameter names: the ArduPilot checkout in thirdparty/ardupilot",
        "# MOT_THST_HOVER, MOT_THST_EXPO: from the Hakoniwa rotor model (SITL); set from the real motor on hardware",
    ] + [f"# {note}" for note in notes] + [f"{key},{format_number(out[key])}" for key in sorted(out)] + [""]
    Path(args.out).write_text("\n".join(lines), encoding="utf-8")
    for note in notes:
        print(("" if note.startswith("WARNING") else "INFO: ") + note, file=sys.stderr)
    print(f"wrote {args.out} ({len(out)} parameters)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
