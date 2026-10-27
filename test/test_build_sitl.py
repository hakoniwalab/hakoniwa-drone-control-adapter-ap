from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class BuildSitlTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "tools").mkdir()
        (self.root / "thirdparty" / "ardupilot").mkdir(parents=True)
        shutil.copy2(ROOT / "tools" / "build-sitl.bash", self.root / "tools" / "build-sitl.bash")
        self.log = self.root / "calls.log"
        self._write_executable(
            self.root / "thirdparty" / "ardupilot" / "waf",
            """#!/usr/bin/env bash
set -eu
printf 'waf:%s|python=%s|ldflags=%s\n' "$*" "$(command -v python)" "${LDFLAGS:-}" >> "${BUILD_SITL_TEST_LOG}"
if [ "${1:-}" = "copter" ]; then
    mkdir -p build/hako-sitl/sitl/bin
    touch build/hako-sitl/sitl/bin/arducopter
    chmod +x build/hako-sitl/sitl/bin/arducopter
fi
""",
        )

    @staticmethod
    def _write_executable(path: Path, contents: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")
        path.chmod(0o755)

    def _environment(self, path: Path) -> dict[str, str]:
        env = os.environ.copy()
        env["BUILD_SITL_TEST_LOG"] = str(self.log)
        env["PATH"] = f"{path}:/usr/bin:/bin"
        env.pop("ARDUPILOT_PYTHON", None)
        return env

    def _run(self, env: dict[str, str]) -> str:
        subprocess.run(
            ["bash", str(self.root / "tools" / "build-sitl.bash")],
            check=True,
            cwd=self.root,
            env=env,
            capture_output=True,
            text=True,
        )
        return self.log.read_text(encoding="utf-8")

    def test_supplied_python_is_used_directly_without_venv_or_pip(self) -> None:
        foundation_bin = self.root / "foundation" / "bin"
        supplied_python = foundation_bin / "python"
        self._write_executable(
            supplied_python,
            """#!/usr/bin/env bash
set -eu
printf 'foundation-python:%s\n' "$*" >> "${BUILD_SITL_TEST_LOG}"
exec "$@"
""",
        )
        trap_bin = self.root / "trap-bin"
        self._write_executable(
            trap_bin / "python3",
            """#!/usr/bin/env bash
printf 'unexpected-python3:%s\n' "$*" >> "${BUILD_SITL_TEST_LOG}"
exit 99
""",
        )
        self._write_executable(
            trap_bin / "pip",
            """#!/usr/bin/env bash
printf 'unexpected-pip:%s\n' "$*" >> "${BUILD_SITL_TEST_LOG}"
exit 99
""",
        )
        env = self._environment(trap_bin)
        env["ARDUPILOT_PYTHON"] = str(supplied_python)

        log = self._run(env)

        self.assertNotIn("unexpected-python3", log)
        self.assertNotIn("unexpected-pip", log)
        self.assertFalse((self.root / "work" / "sitl-venv").exists())
        self.assertEqual(log.count("foundation-python:"), 2)
        self.assertIn("foundation-python:./waf configure --board sitl --out build/hako-sitl", log)
        self.assertIn("foundation-python:./waf copter", log)
        self.assertEqual(log.count(f"python={supplied_python}"), 2)

    def test_default_path_creates_venv_and_installs_future(self) -> None:
        fake_bin = self.root / "fake-bin"
        self._write_executable(
            fake_bin / "python3",
            """#!/usr/bin/env bash
set -eu
if [ "${1:-}" = "-m" ] && [ "${2:-}" = "venv" ]; then
    printf 'venv:%s\n' "$*" >> "${BUILD_SITL_TEST_LOG}"
    target="${4}"
    mkdir -p "${target}/bin"
    cp "$0" "${target}/bin/python"
    printf '#!/usr/bin/env bash\nprintf "pip:%%s\\n" "$*" >> "$BUILD_SITL_TEST_LOG"\n' > "${target}/bin/pip"
    chmod +x "${target}/bin/pip"
    exit 0
fi
printf 'venv-python:%s\n' "$*" >> "${BUILD_SITL_TEST_LOG}"
exec "$@"
""",
        )
        env = self._environment(fake_bin)

        log = self._run(env)

        venv_python = self.root / "work" / "sitl-venv" / "bin" / "python"
        self.assertIn("venv:-m venv --system-site-packages", log)
        self.assertIn("pip:install -q setuptools<80 empy==3.3.4 pexpect future", log)
        self.assertEqual(log.count("venv-python:"), 2)
        self.assertEqual(log.count(f"python={venv_python}"), 2)


if __name__ == "__main__":
    unittest.main()
