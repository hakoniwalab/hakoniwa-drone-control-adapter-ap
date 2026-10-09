#!/usr/bin/env bash
# Build ArduCopter SITL (arducopter with MAVLink) from the pinned ArduPilot checkout.
#
# The adapter's own build (build.bash) configures ArduPilot with --no-gcs for
# the NavEKF3 library, so the vehicle binary is built in a separate waf output
# directory: thirdparty/ardupilot/build-hako-sitl/sitl/bin/arducopter.
#
# Usage: bash tools/build-sitl.bash [--clean]
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ardupilot="${root}/thirdparty/ardupilot"
out="build-hako-sitl"
venv="${root}/work/sitl-venv"
python="${ARDUPILOT_PYTHON:-python3}"

if [ "${1:-}" = "--clean" ]; then
    rm -rf "${ardupilot}/${out}"
fi

# waf's code generators import pkg_resources (setuptools); a venv keeps that
# out of the user's Python.
if [ ! -x "${venv}/bin/python" ]; then
    "${python}" -m venv --system-site-packages "${venv}"
    "${venv}/bin/pip" install -q "setuptools<80" empy==3.3.4 pexpect
fi

ldflags=""
if [ "$(uname)" = "Darwin" ]; then
    # The Xcode 16+ linker rejects ArduPilot's AP_FWVersion object alignment
    # with chained fixups ("pointer not aligned"); the classic layout links.
    ldflags="-Wl,-no_fixup_chains"
fi

(
    cd "${ardupilot}"
    PATH="${venv}/bin:${PATH}" LDFLAGS="${ldflags}" python ./waf configure --board sitl --out "${out}"
    PATH="${venv}/bin:${PATH}" python ./waf copter
)

binary="${ardupilot}/${out}/sitl/bin/arducopter"
test -x "${binary}"
echo "OK: ${binary}"
