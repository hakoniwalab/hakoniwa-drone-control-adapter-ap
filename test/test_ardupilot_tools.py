from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EXPORTER = load_module("export_ardupilot_params", ROOT / "tuning" / "export_ardupilot_params.py")
SETUP = load_module("setup_sitl_runtime", ROOT / "tools" / "setup_sitl_runtime.py")
PREPARE = load_module("prepare_ardupilot_inputs", ROOT / "tuning" / "prepare_ardupilot_inputs.py")


class ExportArduPilotParamsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.runtime = self.base / "runtime"
        (self.runtime / "drone_config").mkdir(parents=True)
        self.params = {
            key: float(index + 1) / 10
            for index, key in enumerate(EXPORTER.PID_TO_ARDUPILOT)
        }
        for index, pair in enumerate(EXPORTER.SHARED_AXIS_PID_TO_ARDUPILOT):
            self.params[pair[0]] = self.params[pair[1]] = 2.0 + index
        self.params.update({
            "PID_ROLL_RPM_MAX": 10.0,
            "PID_PITCH_RPM_MAX": 20.0,
            "PID_YAW_RPM_MAX": 30.0,
            "PID_POS_MAX_ROLL": 25.0,
            "PID_POS_MAX_PITCH": 20.0,
            "PID_POS_MAX_SPD": 4.5,
            "PID_ALT_MAX_SPD": 2.5,
            "ANGULAR_RATE_CONTROL_CYCLE": 0.004,
            "ANGULAR_CONTROL_CYCLE": 0.004,
            "PID_ALT_CONTROL_CYCLE": 0.004,
            "POS_CONTROL_CYCLE": 0.004,
            "SPD_CONTROL_CYCLE": 0.004,
        })
        (self.runtime / "controller.txt").write_text(
            "".join(f"{key} {value}\n" for key, value in self.params.items()), encoding="utf-8"
        )
        adapter_params = {
            "ARMING_DELAY_SEC": 2.0,
            "INS_GYRO_FILTER": 42,
            "PSC_VELZ_IMAX": 1.2,
            "PSC_VELXY_IMAX": 2.3,
            "PSC_ACCZ_IMAX": 0.4,
            "WPNAV_ACCEL_Z": 3.1,
            "PSC_ACCXY_MAX": 2.2,
            "ATC_RATE_FF_ENAB": 1,
            "ATC_INPUT_TC": 0.15,
            "ATC_SLEW_YAW": 6000,
            "GPS1_DELAY_MS": 0.4,
            "EK3_HGT_DELAY": 60,
            "MOT_SPOOL_TIME": 0.5,
            "TKOFF_SLEW_TIME": 2.0,
            "TKOFF_THR_MAX": 0.9,
            "LAND_SPEED": 0.5,
            "LAND_SPEED_HIGH": 0.0,
            "LAND_ALT_LOW": 10.0,
        }
        (self.runtime / "adapter.json").write_text(
            json.dumps({"parameters": adapter_params}), encoding="utf-8"
        )
        self.config = {
            "simulation": {"timeStep": 0.004},
            "controller": {
                "backendType": "adapter-plugin",
                "paramFilePath": "../controller.txt",
                "backendConfig": {"configPath": "../adapter.json"},
            },
            "components": {"thruster": {"rotorPositions": [{}] * 6}},
        }
        self.config_path = self.runtime / "drone_config" / "drone_config_0.json"
        self.config_path.write_text(json.dumps(self.config), encoding="utf-8")

    def run_export(self) -> tuple[dict[str, float], str]:
        out = self.base / "params.parm"
        hover = {
            "inputs": {"K": 1.0, "R": 0.0, "Cq": 0.0, "D": 0.0, "NominalVoltage": 1.0},
            "omega_max_effective": 1.0,
            "hover_duty": 0.5,
        }
        native = types.ModuleType("tuning.px4.conversion.native_params")
        native.compute_hover_thrust_fraction = lambda *_: (None, hover, None)
        modules = {
            "tuning.px4": types.ModuleType("tuning.px4"),
            "tuning.px4.conversion": types.ModuleType("tuning.px4.conversion"),
            "tuning.px4.conversion.native_params": native,
        }
        stderr = io.StringIO()
        argv = ["export", "--pro-root", str(self.base), "--runtime", str(self.runtime), "--out", str(out)]
        with patch.dict(sys.modules, modules), patch.object(sys, "argv", argv), contextlib.redirect_stderr(stderr):
            self.assertEqual(EXPORTER.main(), 0)
        values = {}
        for line in out.read_text(encoding="utf-8").splitlines():
            if line and not line.startswith("#"):
                key, value = line.split(",")
                values[key] = float(value)
        return values, stderr.getvalue()

    def test_exports_pid_mapping_units_vehicle_and_warnings(self) -> None:
        values, stderr = self.run_export()
        self.assertEqual(EXPORTER.FRAME_CLASS_BY_ROTORS, {4: 1, 6: 2, 8: 3})
        for source, target in EXPORTER.PID_TO_ARDUPILOT.items():
            self.assertEqual(values[target], self.params[source])
        for pair, target in EXPORTER.SHARED_AXIS_PID_TO_ARDUPILOT.items():
            self.assertEqual(values[target], self.params[pair[0]])
        expected = {
            "ATC_RATE_R_MAX": 60, "ATC_RATE_P_MAX": 120, "ATC_RATE_Y_MAX": 180,
            "ANGLE_MAX": 2000, "WPNAV_SPEED": 450, "LOIT_SPEED": 450,
            "WPNAV_SPEED_UP": 250, "WPNAV_SPEED_DN": 250, "PILOT_SPEED_UP": 250,
            "WPNAV_ACCEL_Z": 310,
            "WPNAV_ACCEL": 178.466,
            "LOIT_ACC_MAX": 178.466,
            "ATC_RATE_FF_ENAB": 1, "ATC_INPUT_TC": 0.15, "ATC_SLEW_YAW": 6000,
            "PSC_VELZ_IMAX": 120, "PSC_VELXY_IMAX": 230, "PSC_ACCZ_IMAX": 400,
            "INS_GYRO_FILTER": 42, "MOT_PWM_MIN": 1000, "MOT_PWM_MAX": 2000,
            "MOT_SPIN_MIN": 0.15, "MOT_SPIN_MAX": 0.95, "FRAME_CLASS": 2, "FRAME_TYPE": 1,
            "GPS1_DELAY_MS": 1, "EK3_HGT_DELAY": 60,
            "MOT_SPOOL_TIME": 0.5, "TKOFF_SLEW_TIME": 2.0, "TKOFF_THR_MAX": 0.9,
            "LAND_SPEED": 50, "LAND_SPEED_HIGH": 0, "LAND_ALT_LOW": 1000,
        }
        for key, value in expected.items():
            self.assertAlmostEqual(values[key], value)
        self.assertNotIn("ARMING_DELAY_SEC", values)
        self.assertIn("WARNING:", stderr)
        self.assertIn("differs from the SITL period 0.003 s", stderr)

    def test_rejects_unequal_horizontal_axis_gains(self) -> None:
        self.params["PID_POS_Y_Kp"] += 1
        (self.runtime / "controller.txt").write_text(
            "".join(f"{key} {value}\n" for key, value in self.params.items()), encoding="utf-8"
        )
        with self.assertRaisesRegex(SystemExit, "must be equal"):
            self.run_export()

    def test_warns_but_exports_control_gain_outside_ardupilot_range(self) -> None:
        self.params["PID_ALT_Kp"] = 0.5
        (self.runtime / "controller.txt").write_text(
            "".join(f"{key} {value}\n" for key, value in self.params.items()), encoding="utf-8"
        )
        values, stderr = self.run_export()
        self.assertEqual(values["PSC_POSZ_P"], 0.5)
        self.assertIn("WARNING: PSC_POSZ_P=0.5 is outside ArduPilot's range 1..3", stderr)

    def test_horizontal_acceleration_comes_from_half_tilt_limit(self) -> None:
        self.params["PID_POS_MAX_ROLL"] = 25.0
        self.params["PID_POS_MAX_PITCH"] = 25.0
        (self.runtime / "controller.txt").write_text(
            "".join(f"{key} {value}\n" for key, value in self.params.items()), encoding="utf-8"
        )
        values, _ = self.run_export()
        self.assertAlmostEqual(values["WPNAV_ACCEL"], 228.646, places=3)
        self.assertAlmostEqual(values["LOIT_ACC_MAX"], 228.646, places=3)

    def test_rejects_non_adapter_plugin_backend(self) -> None:
        self.config["controller"]["backendType"] = "builtin"
        self.config_path.write_text(json.dumps(self.config), encoding="utf-8")
        with self.assertRaisesRegex(SystemExit, "backendType must be adapter-plugin"):
            self.run_export()

    def test_eams_regression_hover_values(self) -> None:
        plugin = ROOT.parent / "hakoniwa-drone-pro-plugin"
        pro = ROOT.parent / "hakoniwa-drone-pro"
        runtime = plugin / "work" / "pid-tuning" / "eams-ap-sitl" / "runtime" / "all"
        if not (runtime / "drone_config" / "drone_config_0.json").is_file():
            self.skipTest("EAMS tuning runtime is not present")
        out = self.base / "eams.parm"
        argv = ["export", "--pro-root", str(plugin), "--runtime", str(runtime), "--out", str(out)]
        with patch.object(sys, "argv", argv):
            self.assertEqual(EXPORTER.main(), 0)
        values = dict(line.split(",") for line in out.read_text().splitlines() if line and not line.startswith("#"))
        self.assertAlmostEqual(float(values["MOT_THST_HOVER"]), 0.3339, places=4)
        self.assertAlmostEqual(float(values["MOT_THST_EXPO"]), 0.3044, places=4)


class SetupSitlRuntimeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.pro = self.base / "pro"
        self.profile = self.base / "profile"
        self.runtime = self.profile / "runtime" / "all" / "drone_config"
        self.source = self.base / "sitl-source"
        self.runtime.mkdir(parents=True)
        self.source.mkdir()
        tuned = {
            "simulation": {"timeStep": 0.003, "location": {"latitude": 47.641468, "longitude": -122.140165,
                                                          "magneticField": {"declination_deg": 15.16}}},
            "components": {
                "droneDynamics": {"mujoco": {"modelPath": "drone.xml"}},
                "sensors": {
                    "gps": {"delayMsec": 100, "noise": 1,
                            "accuracy": {"horizontalM": 0.2, "verticalM": 0.3, "speedMps": 0.1}},
                    "baro": {"delayMsec": 60, "noise": 0.1},
                    "mag": {"delayMsec": 20, "noise": 10},
                    "gyro": {"noise": 3, "drift": {"mode": "ardupilot-sitl",
                                                     "speedDegPerSecPerMin": 0.07, "timeMin": 4}},
                    "imu": {"noise": 2},
                },
            },
            "controller": {"backendType": "adapter-plugin"},
        }
        (self.runtime / "drone_config_0.json").write_text(json.dumps(tuned), encoding="utf-8")
        (self.runtime / "drone.xml").write_text("model", encoding="utf-8")
        sitl = {"simulation": {"sitl": {"actuator_to_rotor_index": [0, 1, 2, 3]}}, "controller": {"backendType": "sitl"}}
        (self.source / "drone_config_0.json").write_text(json.dumps(sitl), encoding="utf-8")
        self.service = self.base / "aircraft_service_ardupilot"
        self.service.write_text("binary", encoding="utf-8")
        self.out = self.base / "out"

    def run_setup(
        self,
        service: Path | None = None,
        gps_hz: float | None = 7.0,
        *,
        no_service_script: bool = False,
    ) -> int:
        argv = ["setup", "--pro-root", str(self.pro), "--profile", str(self.profile),
                "--sitl-drone-config", str(self.source), "--out", str(self.out)]
        if gps_hz is not None:
            argv += ["--gps-hz", str(gps_hz)]
        if service is not None:
            argv += ["--service", str(service)]
        if no_service_script:
            argv.append("--no-service-script")

        def fake_export(command, check):
            Path(command[command.index("--out") + 1]).write_text(
                "FRAME_CLASS,1\nINS_ACCEL_FILTER,10\n", encoding="utf-8")

        with patch.object(sys, "argv", argv), patch.object(SETUP.subprocess, "run", side_effect=fake_export):
            return SETUP.main()

    def test_creates_complete_runtime(self) -> None:
        self.assertEqual(self.run_setup(self.service), 0)
        self.assertTrue((self.out / "params.parm").is_file())
        sitl_values = [line for line in (self.out / "sitl.parm").read_text().splitlines()
                       if line and not line.startswith("#")]
        self.assertEqual(sitl_values, ["ARMING_CHECK,0", "COMPASS_DEC,0.264591915", "SIM_SPEEDUP,1", "LOG_BITMASK,180223", "SIM_GPS_HZ,7",
                                       "INS_ACCEL_FILTER,30", "SIM_GPS_LAG_MS,100",
                                       "SIM_BARO_DELAY,60", "SIM_MAG_DELAY,20",
                                       "SIM_BARO_RND,0.173205", "SIM_MAG_RND,0.173205",
                                       "SIM_GPS_NOISE,0", "SIM_BARO_COUNT,2",
                                       "SIM_DRIFT_SPEED,0.07", "SIM_DRIFT_TIME,4"])
        self.assertNotIn("LOG_BITMASK", (self.out / "params.parm").read_text())
        config = json.loads((self.out / "drone_config" / "drone_config_0.json").read_text())
        self.assertEqual(config["simulation"]["timeStep"], 0.003)
        self.assertEqual(config["simulation"]["mavlink_tx_period_msec"]["hil_sensor"], 3)
        self.assertIn("sitl", config["simulation"])
        self.assertEqual(config["components"]["sensors"]["gps"], {
            "noise": 1,
            "accuracy": {"horizontalM": 0.2, "verticalM": 0.3, "speedMps": 0.1},
        })
        self.assertEqual(config["components"]["sensors"]["baro"], {"noise": 0.1})
        self.assertEqual(config["components"]["sensors"]["mag"], {"noise": 10})
        self.assertEqual(config["components"]["sensors"]["gyro"], {"noise": 3})
        self.assertEqual(config["components"]["sensors"]["imu"], {"noise": 2})
        self.assertTrue((self.out / "drone_config" / "drone.xml").is_file())
        for name in ("run-service.bash", "run-arducopter.bash"):
            self.assertTrue((self.out / name).stat().st_mode & 0o111)
        marker = json.loads((self.out / "ardupilot-sitl-runtime.json").read_text())
        self.assertTrue(marker["service_script"])
        self.assertEqual(marker["aircraft_service_sensor_delay_msec_removed"], ["gps", "baro", "mag"])
        self.assertEqual(marker["aircraft_service_sensor_delay_msec_removed_values"],
                         {"gps": 100, "baro": 60, "mag": 20})
        self.assertIn("GPS/barometer/magnetometer", marker["aircraft_service_sensor_delay_reason"])
        self.assertIn("apply it twice", marker["aircraft_service_sensor_delay_reason"])
        self.assertTrue(marker["aircraft_service_gyro_drift_removed"])
        self.assertEqual(marker["aircraft_service_gyro_drift_removed_value"]["timeMin"], 4)
        self.assertIn("apply it twice", marker["aircraft_service_gyro_drift_reason"])
        self.assertEqual(marker["sensor_parameter_conversions"]["SIM_MAG_RND"]["source"],
                         "components.sensors.mag.noise * sqrt(3) / 100")
        self.assertEqual(marker["compass_declination"]["source"],
                         "simulation.location.magneticField.declination_deg")
        self.assertTrue(marker["gps_accuracy"]["configured"])
        self.assertFalse(marker["gps_accuracy"]["used_by_ardupilot_sitl"])
        self.assertIn("ArduPilot SITL creates its own GPS", marker["gps_accuracy"]["reason"])

    def test_sensor_model_v2_provenance_is_copied_to_sitl_runtime(self) -> None:
        provenance = {
            "ardupilot_sitl_parameters": {
                "SIM_MAG_RND": 0.5,
                "SIM_BARO_DRIFT": 0,
                "SIM_GPS_VERR_X": math.sqrt(3) * 0.1,
            },
            "ardupilot_sitl_not_applied": [
                {"sensor": "mag", "field": "resolutionNT",
                 "reason": "ArduPilot SITL has no equivalent parameter"},
            ],
        }
        (self.runtime / "sensor-model-v2-provenance.json").write_text(
            json.dumps(provenance), encoding="utf-8"
        )
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        tuned["components"]["sensors"]["gyro"].update({
            "zeroRateOffset": {"fixedDegPerSec": [1, 2, 3]},
            "sensitivity": {"scaleErrorPct": [1, 2, 3]},
        })
        tuned["components"]["sensors"].setdefault("acc", {}).update({
            "ardupilotSitlErrorSource": "ardupilot-sitl",
            "zeroGOffset": {"fixedMg": [20, 0, 0]},
            "sensitivity": {"scaleErrorPct": [1, 0, 0]},
        })
        tuned_path.write_text(json.dumps(tuned))
        self.assertEqual(self.run_setup(self.service), 0)
        values = (self.out / "sitl.parm").read_text(encoding="utf-8")
        self.assertEqual(1, values.count("SIM_MAG_RND,"))
        self.assertIn("SIM_MAG_RND,0.5\n", values)
        self.assertIn("SIM_BARO_DRIFT,0\n", values)
        self.assertIn(f"SIM_GPS_VERR_X,{math.sqrt(3) * 0.1:g}\n", values)
        record = (self.out / "ardupilot-sitl-not-applied.txt").read_text(encoding="utf-8")
        self.assertIn("mag.resolutionNT: ArduPilot SITL has no equivalent parameter", record)
        runtime_config = json.loads((self.out / "drone_config/drone_config_0.json").read_text())
        runtime_gyro = runtime_config["components"]["sensors"]["gyro"]
        self.assertNotIn("zeroRateOffset", runtime_gyro)
        self.assertNotIn("sensitivity", runtime_gyro)
        runtime_acc = runtime_config["components"]["sensors"]["acc"]
        self.assertNotIn("zeroGOffset", runtime_acc)
        self.assertNotIn("sensitivity", runtime_acc)

    def test_missing_sensor_model_v2_provenance_keeps_legacy_runtime(self) -> None:
        self.assertEqual(self.run_setup(self.service), 0)
        self.assertFalse((self.out / "ardupilot-sitl-not-applied.txt").exists())
        self.assertIn("SIM_MAG_RND,0.173205\n", (self.out / "sitl.parm").read_text())

    def test_sensor_conversion_warnings_and_drift_modes(self) -> None:
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        tuned["components"]["sensors"]["gyro"]["drift"] = {"mode": "random-walk"}
        tuned_path.write_text(json.dumps(tuned))
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(self.service), 0)
        self.assertIn("GPS noise is nonzero", stderr.getvalue())
        self.assertIn("random-walk drift cannot be reproduced", stderr.getvalue())
        values = (self.out / "sitl.parm").read_text()
        self.assertIn("SIM_DRIFT_SPEED,0\n", values)
        self.assertNotIn("SIM_DRIFT_TIME", values)

    def test_sensor_update_periods_configure_gps_and_record_unavailable_sensors(self) -> None:
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        tuned["simulation"]["timeStep"] = 0.001
        sensors = tuned["components"]["sensors"]
        sensors.setdefault("acc", {})["updatePeriodMsec"] = 4
        sensors["gyro"]["updatePeriodMsec"] = 4
        sensors["baro"]["updatePeriodMsec"] = 20
        sensors["mag"]["updatePeriodMsec"] = 100
        sensors["gps"]["updatePeriodMsec"] = 100
        tuned_path.write_text(json.dumps(tuned))

        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(self.service, gps_hz=7.0), 0)

        self.assertIn("SIM_GPS_HZ,10\n", (self.out / "sitl.parm").read_text())
        self.assertIn("--gps-hz 7 is overridden", stderr.getvalue())
        self.assertIn("baro.updatePeriodMsec is unused", stderr.getvalue())
        self.assertIn("mag.updatePeriodMsec is unused", stderr.getvalue())
        marker = json.loads((self.out / "ardupilot-sitl-runtime.json").read_text())
        self.assertTrue(marker["sensor_update_periods"]["gps"]["used_by_ardupilot_sitl"])
        self.assertEqual(marker["sensor_update_periods"]["gps"]["effective_hz"], 10.0)
        self.assertFalse(marker["sensor_update_periods"]["baro"]["used_by_ardupilot_sitl"])
        self.assertFalse(marker["sensor_update_periods"]["mag"]["used_by_ardupilot_sitl"])
        self.assertFalse(marker["sensor_update_periods"]["acc"]["used_by_ardupilot_sitl"])

    def test_unset_update_period_keeps_default_gps_rate(self) -> None:
        self.assertEqual(self.run_setup(self.service, gps_hz=None), 0)
        self.assertIn("SIM_GPS_HZ,5\n", (self.out / "sitl.parm").read_text())
        marker = json.loads((self.out / "ardupilot-sitl-runtime.json").read_text())
        self.assertFalse(marker["sensor_update_periods"]["gps"]["configured"])

    def test_update_period_requires_json_integer_step_multiple_and_bounded_value(self) -> None:
        base = {"simulation": {"timeStep": 0.003}, "components": {"sensors": {"gps": {}}}}
        for value in (True, "100", 100.0, 0, -1):
            config = json.loads(json.dumps(base))
            config["components"]["sensors"]["gps"]["updatePeriodMsec"] = value
            with self.assertRaisesRegex(RuntimeError, "must be a positive integer"):
                SETUP.sensor_update_period_msec(config, "gps")

        config = json.loads(json.dumps(base))
        config["components"]["sensors"]["gps"]["updatePeriodMsec"] = 100
        with self.assertRaisesRegex(RuntimeError, "multiple of simulation.timeStep"):
            SETUP.sensor_update_period_msec(config, "gps")

        config["components"]["sensors"]["gps"]["updatePeriodMsec"] = 3_600_001
        with self.assertRaisesRegex(RuntimeError, "must not exceed 3600000 ms"):
            SETUP.sensor_update_period_msec(config, "gps")

    def test_non_integral_gps_frequency_rounds_to_ap_int8_and_warns(self) -> None:
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        tuned["components"]["sensors"]["gps"]["updatePeriodMsec"] = 150
        tuned_path.write_text(json.dumps(tuned))

        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(self.service), 0)

        self.assertIn("SIM_GPS_HZ,7\n", (self.out / "sitl.parm").read_text())
        self.assertIn("150 ms cannot be represented exactly", stderr.getvalue())
        self.assertIn("using 7 Hz", stderr.getvalue())
        marker = json.loads((self.out / "ardupilot-sitl-runtime.json").read_text())
        self.assertEqual(marker["sensor_update_periods"]["gps"]["effective_hz"], 7)

        self.assertEqual(SETUP.ardupilot_gps_hz(1)[0], 127)
        self.assertIn("AP_Int8", SETUP.ardupilot_gps_hz(1)[1])
        self.assertEqual(SETUP.ardupilot_gps_hz(3_600_000)[0], 1)
        self.assertIn("AP_Int8", SETUP.ardupilot_gps_hz(3_600_000)[1])

    def test_no_drift_forces_zero_and_declination_mismatch_warns(self) -> None:
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        del tuned["components"]["sensors"]["gyro"]["drift"]
        tuned["simulation"]["location"]["magneticField"]["declination_deg"] = 16
        tuned_path.write_text(json.dumps(tuned))
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(self.service), 0)
        self.assertIn("differs from ArduPilot's table", stderr.getvalue())
        self.assertIn("COMPASS_DEC,0.27925268", (self.out / "sitl.parm").read_text())

    def test_missing_default_service_is_an_error(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(), 1)
        self.assertIn("aircraft service was not found", stderr.getvalue())

    def test_no_service_script_skips_service_check_and_script(self) -> None:
        self.assertEqual(self.run_setup(no_service_script=True), 0)

        self.assertFalse((self.out / "run-service.bash").exists())
        self.assertTrue((self.out / "run-arducopter.bash").stat().st_mode & 0o111)
        self.assertTrue((self.out / "params.parm").is_file())
        self.assertTrue((self.out / "sitl.parm").is_file())
        self.assertTrue((self.out / "drone_config" / "drone_config_0.json").is_file())
        marker = json.loads((self.out / "ardupilot-sitl-runtime.json").read_text())
        self.assertFalse(marker["service_script"])

    def test_one_millisecond_step_keeps_accel_filter_and_submillisecond_period_is_clamped(self) -> None:
        self.assertEqual(SETUP.sitl_accel_filter_hz(10, 0.001), 10)
        tuned_path = self.runtime / "drone_config_0.json"
        tuned = json.loads(tuned_path.read_text())
        tuned["simulation"]["timeStep"] = 0.0004
        tuned_path.write_text(json.dumps(tuned))
        self.assertEqual(self.run_setup(self.service), 0)
        config = json.loads((self.out / "drone_config" / "drone_config_0.json").read_text())
        self.assertEqual(config["simulation"]["mavlink_tx_period_msec"]["hil_sensor"], 1)


class PrepareArduPilotInputsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.source = self.base / "source"
        self.output = self.base / "output"
        self.source.mkdir()

    def run_prepare(self, sensors: dict | None = None, prop_inch: int | None = None,
                    model_xml: str | None = None, sensor_preset: str | None = None) -> tuple[dict, dict]:
        source_config = {
            "simulation": {"location": {"latitude": 47.641468, "longitude": -122.140165}},
            "components": {"sensors": sensors or {}},
        }
        (self.source / "drone_config_0.json").write_text(json.dumps(source_config))
        (self.source / "controller-params.txt").write_text("")
        (self.source / "drone.xml").write_text(model_xml or (
            '<mujoco><worldbody><body><geom name="prop1_geom" type="cylinder" '
            'size="0.127 0.0015"/></body></worldbody></mujoco>'))

        def fake_px4_prepare(command, check, cwd):
            self.output.mkdir()
            for name in ("drone_config_0.json", "controller-params.txt", "drone.xml"):
                (self.output / name).write_bytes((self.source / name).read_bytes())

        native = types.ModuleType("tuning.px4.conversion.native_params")
        native.compute_hover_thrust_fraction = lambda *_: None
        modules = {
            "tuning.px4": types.ModuleType("tuning.px4"),
            "tuning.px4.conversion": types.ModuleType("tuning.px4.conversion"),
            "tuning.px4.conversion.native_params": native,
        }
        argv = ["prepare", "--pro-root", str(self.base), str(self.source), str(self.output)]
        if prop_inch is not None:
            argv[1:1] = ["--prop-inch", str(prop_inch)]
        if sensor_preset is not None:
            argv[1:1] = ["--sensor-preset", sensor_preset]
        model = {
            "thrust_hover": 0.42, "hover_duty": 0.5,
            "spin_arm": 0.10, "spin_min": 0.15,
            "motor_model_a_coef": 0.2, "motor_model_b_coef": 0.3,
            "motor_nominal_voltage": 12.0, "motor_hover_omega": 400.0,
            "spin_arm_thrust_hover": 0.04, "spin_min_thrust_hover": 0.09,
        }
        stdout = io.StringIO()
        with patch.dict(sys.modules, modules), patch.object(sys, "argv", argv), \
             patch.object(PREPARE.subprocess, "run", side_effect=fake_px4_prepare), \
             patch.object(PREPARE, "motor_thrust_model", return_value=model), \
             contextlib.redirect_stdout(stdout):
            self.assertEqual(PREPARE.main(), 0)
        self.prepare_stdout = stdout.getvalue()
        drone = json.loads((self.output / "drone_config_0.json").read_text())
        adapter = json.loads((self.output / "ardupilot-controller-config.json").read_text())
        return drone, adapter

    def test_missing_baro_delay_is_fixed_to_plugin_nav_ekf_default(self) -> None:
        drone, adapter = self.run_prepare()
        self.assertNotIn("delayMsec", drone["components"]["sensors"]["gps"])
        self.assertEqual(drone["components"]["sensors"]["baro"]["delayMsec"], 60)
        self.assertEqual(adapter["parameters"]["GPS1_DELAY_MS"], 0)
        self.assertEqual(adapter["parameters"]["EK3_HGT_DELAY"], 60)
        self.assertIn("plugin's NavEKF3 fixes EK3_HGT_DELAY at its 60 ms default", self.prepare_stdout)
        self.assertEqual(adapter["parameters"]["MOT_SPIN_ARM"], 0.10)
        self.assertEqual(adapter["parameters"]["MOT_SPIN_MIN"], 0.15)
        self.assertEqual(adapter["parameters"]["MOT_SPIN_MAX"], 0.95)
        self.assertEqual(adapter["parameters"]["motor_model_a_coef"], 0.2)
        self.assertEqual(adapter["parameters"]["motor_model_b_coef"], 0.3)
        self.assertEqual(adapter["parameters"]["motor_nominal_voltage"], 12.0)
        self.assertEqual(adapter["parameters"]["motor_hover_omega"], 400.0)
        self.assertEqual(adapter["parameters"]["takeoff_slew_loop_rate_hz"], 400.0)
        self.assertAlmostEqual(drone["simulation"]["location"]["magneticField"]["declination_deg"], 15.15504, places=5)
        self.assertNotIn("gyro", drone["components"]["sensors"])
        self.assertNotIn("mag", drone["components"]["sensors"])
        self.assertNotIn("noise", drone["components"]["sensors"]["baro"])

    def test_non_default_baro_delay_is_rejected_before_tuning(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit):
            self.run_prepare({"baro": {"delayMsec": 59}})
        self.assertIn("EK3_HGT_DELAY must be 60", stderr.getvalue())
        self.assertIn("set the barometer delay to 60 ms instead", stderr.getvalue())

    def test_sensor_characteristics_are_preserved_without_preset(self) -> None:
        original = {
            "acc": {"noise": 0.03}, "gyro": {"noise": 0.04},
            "mag": {"noise": 0.03}, "baro": {"noise": 0.01}, "gps": {"noise": 0.05},
        }
        drone, _ = self.run_prepare(original)
        sensors = drone["components"]["sensors"]
        self.assertEqual(sensors["acc"]["noise"], 0.03)
        self.assertEqual(sensors["gyro"]["noise"], 0.04)
        self.assertEqual(sensors["gps"]["noise"], 0.05)
        self.assertEqual(sensors["mag"]["noise"], 0.03)
        self.assertEqual(sensors["baro"]["noise"], 0.01)

    def test_explicit_sensor_preset_applies_ardupilot_defaults(self) -> None:
        drone, adapter = self.run_prepare(sensor_preset="ardupilot-sitl-default")
        sensors = drone["components"]["sensors"]
        self.assertEqual(sensors["gps"]["delayMsec"], 100)
        self.assertEqual(sensors["baro"]["delayMsec"], 60)
        self.assertEqual(sensors["gyro"]["drift"], {
            "mode": "ardupilot-sitl", "speedDegPerSecPerMin": 0.05, "timeMin": 5.0})
        self.assertEqual(sensors["mag"]["noise"], 0)
        self.assertAlmostEqual(sensors["baro"]["noise"], 0.2 / math.sqrt(3.0))
        self.assertEqual(adapter["parameters"]["GPS1_DELAY_MS"], 100)

    def test_preserves_existing_sensor_delays(self) -> None:
        drone, adapter = self.run_prepare({"gps": {"delayMsec": 25}, "baro": {"delayMsec": 60}})
        self.assertEqual(drone["components"]["sensors"]["gps"]["delayMsec"], 25)
        self.assertEqual(adapter["parameters"]["GPS1_DELAY_MS"], 25)

    def test_x500_propeller_model_produces_mission_planner_initial_parameters(self) -> None:
        _, adapter = self.run_prepare()
        expected = {
            "INS_GYRO_FILTER": 42, "INS_ACCEL_FILTER": 20,
            "ATC_RAT_RLL_FLTD": 21, "ATC_RAT_RLL_FLTT": 21, "ATC_RAT_RLL_FLTE": 0,
            "ATC_RAT_PIT_FLTD": 21, "ATC_RAT_PIT_FLTT": 21, "ATC_RAT_PIT_FLTE": 0,
            "ATC_RAT_YAW_FLTD": 0, "ATC_RAT_YAW_FLTE": 2, "ATC_RAT_YAW_FLTT": 21,
            "ATC_ACCEL_R_MAX": 116700, "ATC_ACCEL_P_MAX": 116700,
            "ATC_ACCEL_Y_MAX": 27000,
        }
        for key, value in expected.items():
            self.assertEqual(adapter["parameters"][key], value)
        notes = adapter["initial_parameters"]
        self.assertEqual(notes["propeller_diameter_inch"], 10)
        self.assertIn("InitialParamsCalculator.cs", notes["source"])
        self.assertIn("INS_GYRO_FILTER", notes["formulas"])

    def test_explicit_propeller_size_is_used_when_model_has_no_propeller(self) -> None:
        _, adapter = self.run_prepare(prop_inch=10, model_xml="<mujoco/>")
        self.assertEqual(adapter["parameters"]["INS_GYRO_FILTER"], 42)

    def test_missing_propeller_size_is_an_error(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit):
            self.run_prepare(model_xml="<mujoco/>")
        self.assertIn("no named propeller cylinder radius", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
