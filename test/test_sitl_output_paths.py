from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_setup_sitl_runtime():
    path = ROOT / "tools" / "setup_sitl_runtime.py"
    spec = importlib.util.spec_from_file_location("setup_sitl_runtime_paths", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SitlOutputPathsTest(unittest.TestCase):
    def test_arducopter_is_in_ignored_ardupilot_build_directory(self) -> None:
        setup = load_setup_sitl_runtime()
        expected = ROOT / "thirdparty" / "ardupilot" / "build" / "hako-sitl" / "sitl" / "bin" / "arducopter"
        self.assertEqual(setup.ARDUCOPTER, expected)

    def test_build_script_uses_ignored_ardupilot_build_directory(self) -> None:
        script = (ROOT / "tools" / "build-sitl.bash").read_text(encoding="utf-8")
        self.assertIn('out="build/hako-sitl"', script)


if __name__ == "__main__":
    unittest.main()
