#!/usr/bin/env bash

set -euo pipefail

readonly project_root="$(cd "$(dirname "$0")" && pwd)"
readonly ardupilot_root="${project_root}/thirdparty/ardupilot"
readonly build_root="${project_root}/build"
readonly action="${1:-build}"
readonly ardupilot_python="${ARDUPILOT_PYTHON:-python3}"

configure_ardupilot() {
    (
        cd "${ardupilot_root}"
        "${ardupilot_python}" ./waf configure --board sitl
    )
}

ensure_ardupilot_configured() {
    if [[ ! -f "${ardupilot_root}/build/sitl/ap_config.h" ]]; then
        configure_ardupilot
    fi
}

configure_adapter() {
    cmake -S "${project_root}" -B "${build_root}" -DBUILD_TESTING=ON
}

case "${action}" in
    ardupilot)
        configure_ardupilot
        ;;
    build)
        ensure_ardupilot_configured
        configure_adapter
        cmake --build "${build_root}"
        ;;
    test)
        ensure_ardupilot_configured
        configure_adapter
        cmake --build "${build_root}"
        ctest --test-dir "${build_root}" --output-on-failure
        ;;
    *)
        echo "Usage: $0 {ardupilot|build|test}" >&2
        exit 1
        ;;
esac
