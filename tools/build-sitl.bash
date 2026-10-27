#!/usr/bin/env bash
# Build ArduCopter SITL (arducopter with MAVLink) from the pinned ArduPilot checkout.
#
# The adapter's own build (build.bash) configures ArduPilot with --no-gcs for
# the NavEKF3 library, so the vehicle binary is built in a separate waf output
# directory: thirdparty/ardupilot/build/hako-sitl/sitl/bin/arducopter.
#
# Usage: bash tools/build-sitl.bash [--clean]
#   ARDUPILOT_PYTHON=/path/to/python bash tools/build-sitl.bash [--clean]
#       Use the supplied Python and its installed packages directly.
#   bash tools/build-sitl.bash [--clean]
#       Create work/sitl-venv and install the required Python packages there.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ardupilot="${root}/thirdparty/ardupilot"
out="build/hako-sitl"
venv="${root}/work/sitl-venv"
python="${ARDUPILOT_PYTHON:-python3}"

if [ "${1:-}" = "--clean" ]; then
    rm -rf "${ardupilot}/${out}"
fi

# A supplied Python (for example, Drone PRO's Foundation Python) already has
# the recipe-managed packages. Otherwise, isolate waf's dependencies in a venv.
if [ -n "${ARDUPILOT_PYTHON:-}" ]; then
    python_path="$(command -v "${python}")"
    python_dir="$(cd "$(dirname "${python_path}")" && pwd)"
    python="${python_dir}/$(basename "${python_path}")"
else
    if [ ! -x "${venv}/bin/python" ]; then
        "${python}" -m venv --system-site-packages "${venv}"
        "${venv}/bin/pip" install -q "setuptools<80" empy==3.3.4 pexpect future
    fi
    python_dir="${venv}/bin"
    python="${venv}/bin/python"
fi

ldflags=""
if [ "$(uname)" = "Darwin" ]; then
    # The Xcode 16+ linker rejects ArduPilot's AP_FWVersion object alignment
    # with chained fixups ("pointer not aligned"); the classic layout links.
    ldflags="-Wl,-no_fixup_chains"
fi

(
    cd "${ardupilot}"
    PATH="${python_dir}:${PATH}" LDFLAGS="${ldflags}" "${python}" ./waf configure --board sitl --out "${out}"
    PATH="${python_dir}:${PATH}" "${python}" ./waf copter
)

binary="${ardupilot}/${out}/sitl/bin/arducopter"
test -x "${binary}"
echo "OK: ${binary}"
