from __future__ import annotations

import contextlib
import importlib.util
import io
import json
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
            "INS_GYRO_FILTER": 42,
            "PSC_VELZ_IMAX": 1.2,
            "PSC_VELXY_IMAX": 2.3,
            "PSC_ACCZ_IMAX": 0.4,
            "WPNAV_ACCEL_Z": 3.1,
            "PSC_ACCXY_MAX": 2.2,
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
            "WPNAV_ACCEL_Z": 310, "WPNAV_ACCEL": 220, "LOIT_ACC_MAX": 220,
            "PSC_VELZ_IMAX": 120, "PSC_VELXY_IMAX": 230, "PSC_ACCZ_IMAX": 400,
            "INS_GYRO_FILTER": 42, "MOT_PWM_MIN": 1000, "MOT_PWM_MAX": 2000,
            "MOT_SPIN_MIN": 0.15, "MOT_SPIN_MAX": 0.95, "FRAME_CLASS": 2, "FRAME_TYPE": 1,
        }
        for key, value in expected.items():
            self.assertAlmostEqual(values[key], value)
        self.assertIn("WARNING:", stderr)
        self.assertIn("differs from the SITL period 0.003 s", stderr)

    def test_rejects_unequal_horizontal_axis_gains(self) -> None:
        self.params["PID_POS_Y_Kp"] += 1
        (self.runtime / "controller.txt").write_text(
            "".join(f"{key} {value}\n" for key, value in self.params.items()), encoding="utf-8"
        )
        with self.assertRaisesRegex(SystemExit, "must be equal"):
            self.run_export()

    def test_rejects_non_adapter_plugin_backend(self) -> None:
        self.config["controller"]["backendType"] = "builtin"
        self.config_path.write_text(json.dumps(self.config), encoding="utf-8")
        with self.assertRaisesRegex(SystemExit, "backendType must be adapter-plugin"):
            self.run_export()

    def test_eams_regression_hover_values(self) -> None:
        plugin = ROOT.parent / "hakoniwa-drone-pro-plugin"
        pro = ROOT.parent / "hakoniwa-drone-pro"
        runtime = plugin / "work" / "pid-tuning" / "eams-ap-sitl" / "runtime" / "all"
        if not runtime.is_dir():
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
            "simulation": {"timeStep": 0.003},
            "components": {"droneDynamics": {"mujoco": {"modelPath": "drone.xml"}}},
            "controller": {"backendType": "adapter-plugin"},
        }
        (self.runtime / "drone_config_0.json").write_text(json.dumps(tuned), encoding="utf-8")
        (self.runtime / "drone.xml").write_text("model", encoding="utf-8")
        sitl = {"simulation": {"sitl": {"actuator_to_rotor_index": [0, 1, 2, 3]}}, "controller": {"backendType": "sitl"}}
        (self.source / "drone_config_0.json").write_text(json.dumps(sitl), encoding="utf-8")
        self.service = self.base / "aircraft_service_ardupilot"
        self.service.write_text("binary", encoding="utf-8")
        self.out = self.base / "out"

    def run_setup(self, service: Path | None = None) -> int:
        argv = ["setup", "--pro-root", str(self.pro), "--profile", str(self.profile),
                "--sitl-drone-config", str(self.source), "--out", str(self.out), "--gps-hz", "7"]
        if service is not None:
            argv += ["--service", str(service)]

        def fake_export(command, check):
            Path(command[command.index("--out") + 1]).write_text("FRAME_CLASS,1\n", encoding="utf-8")

        with patch.object(sys, "argv", argv), patch.object(SETUP.subprocess, "run", side_effect=fake_export):
            return SETUP.main()

    def test_creates_complete_runtime(self) -> None:
        self.assertEqual(self.run_setup(self.service), 0)
        self.assertTrue((self.out / "params.parm").is_file())
        self.assertEqual((self.out / "sitl.parm").read_text().splitlines()[1:],
                         ["ARMING_CHECK,0", "LOG_BACKEND_TYPE,0", "SIM_GPS_HZ,7"])
        config = json.loads((self.out / "drone_config" / "drone_config_0.json").read_text())
        self.assertEqual(config["simulation"]["timeStep"], 0.003)
        self.assertIn("sitl", config["simulation"])
        self.assertTrue((self.out / "drone_config" / "drone.xml").is_file())
        for name in ("run-service.bash", "run-arducopter.bash"):
            self.assertTrue((self.out / name).stat().st_mode & 0o111)
        self.assertTrue((self.out / "ardupilot-sitl-runtime.json").is_file())

    def test_missing_default_service_is_an_error(self) -> None:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            self.assertEqual(self.run_setup(), 1)
        self.assertIn("aircraft service was not found", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
