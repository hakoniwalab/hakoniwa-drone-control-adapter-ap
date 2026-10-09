#!/usr/bin/env bash

set -euo pipefail

readonly project_root="$(cd "$(dirname "$0")" && pwd)"
readonly ardupilot_root="${project_root}/thirdparty/ardupilot"
readonly build_root="${project_root}/build"
readonly action="${1:-build}"
readonly install_root="${INSTALL_PREFIX:-${project_root}/install}"
readonly ardupilot_python="${ARDUPILOT_PYTHON:-python3}"
readonly ardupilot_patch="${project_root}/patches/ardupilot-4.6.3-dal-standalone-zero-imu-offset.patch"
ardupilot_patch_applied=0

restore_ardupilot_patch() {
    if [[ "${ardupilot_patch_applied}" -eq 1 ]]; then
        git -C "${ardupilot_root}" apply -R "${ardupilot_patch}"
        ardupilot_patch_applied=0
    fi
}

apply_ardupilot_patch() {
    if git -C "${ardupilot_root}" apply --check "${ardupilot_patch}"; then
        git -C "${ardupilot_root}" apply "${ardupilot_patch}"
        ardupilot_patch_applied=1
        return
    fi
    if git -C "${ardupilot_root}" apply -R --check "${ardupilot_patch}"; then
        # Permit an explicitly pre-patched local checkout without undoing the
        # user's change when this script exits.
        return
    fi
    echo "ArduPilot DAL patch cannot be applied cleanly: ${ardupilot_patch}" >&2
    exit 1
}

trap restore_ardupilot_patch EXIT

configure_ardupilot() {
    (
        cd "${ardupilot_root}"
        "${ardupilot_python}" ./waf configure \
            --board sitl \
            --disable-scripting \
            --no-gcs \
            --enable-EKF3
        "${ardupilot_python}" ./waf build --targets tool/AP_DAL_Standalone
    )
}

ensure_ardupilot_configured() {
    if [[ ! -f "${ardupilot_root}/build/sitl/ap_config.h" \
        || ! -f "${ardupilot_root}/build/sitl/lib/libAP_DAL_libs.a" ]]; then
        configure_ardupilot
    fi
}

configure_adapter() {
    local -a cmake_args=(
        -S "${project_root}"
        -B "${build_root}"
        -DBUILD_TESTING=ON
        -DCMAKE_BUILD_TYPE=Release
        "-DCMAKE_INSTALL_PREFIX=${install_root}"
        -DHAKONIWA_AP_ENABLE_EKF3=ON)
    if [[ "$(uname -s)" == "Darwin" ]]; then
        # Waf uses the current macOS SDK version when no deployment target is
        # supplied. Match it here so the DAL archive and adapter executables
        # carry the same minimum OS version.
        cmake_args+=(
            "-DCMAKE_OSX_DEPLOYMENT_TARGET=${MACOSX_DEPLOYMENT_TARGET:-$(sw_vers -productVersion)}")
    fi
    cmake "${cmake_args[@]}"
}

case "${action}" in
    ardupilot)
        configure_ardupilot
        ;;
    build)
        ensure_ardupilot_configured
        apply_ardupilot_patch
        configure_adapter
        cmake --build "${build_root}"
        ;;
    test)
        ensure_ardupilot_configured
        apply_ardupilot_patch
        configure_adapter
        cmake --build "${build_root}"
        ctest --test-dir "${build_root}" --output-on-failure
        ;;
    clean)
        cmake -E remove_directory "${build_root}"
        ;;
    install)
        ensure_ardupilot_configured
        apply_ardupilot_patch
        configure_adapter
        cmake --build "${build_root}"
        cmake --install "${build_root}"
        ;;
    *)
        echo "Usage: $0 {ardupilot|build|clean|test|install}" >&2
        exit 1
        ;;
esac
