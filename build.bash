#!/usr/bin/env bash

set -euo pipefail

readonly project_root="$(cd "$(dirname "$0")" && pwd)"
readonly ardupilot_root="${project_root}/thirdparty/ardupilot"
readonly build_root="${project_root}/build"
readonly action="${1:-build}"
readonly install_root="${INSTALL_PREFIX:-${project_root}/install}"
readonly ardupilot_python="${ARDUPILOT_PYTHON:-python3}"
readonly ardupilot_patch="${project_root}/patches/ardupilot-4.6.3-dal-standalone-zero-imu-offset.patch"
readonly ardupilot_pic_stamp="${ardupilot_root}/build/sitl/.hakoniwa-dal-pic"
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
        local -a waf_environment=(env)
        if [[ "$(uname -s)" == "Linux" ]]; then
            # libAP_DAL_libs.a is linked into the adapter plugin.  GNU ld
            # requires every object in that archive to be position independent.
            waf_environment+=("CFLAGS=${CFLAGS:-} -fPIC" "CXXFLAGS=${CXXFLAGS:-} -fPIC")
        fi
        "${waf_environment[@]}" "${ardupilot_python}" ./waf configure \
            --board sitl \
            --disable-scripting \
            --no-gcs \
            --enable-EKF3
        "${ardupilot_python}" ./waf build --targets tool/AP_DAL_Standalone
        if [[ "$(uname -s)" == "Linux" ]]; then
            printf '%s\n' 'linux-fpic-v1' > "${ardupilot_pic_stamp}"
        fi
    )
}

ensure_ardupilot_configured() {
    local needs_configure=0
    if [[ ! -f "${ardupilot_root}/build/sitl/ap_config.h" \
        || ! -f "${ardupilot_root}/build/sitl/lib/libAP_DAL_libs.a" ]]; then
        needs_configure=1
    elif [[ "$(uname -s)" == "Linux" \
        && ( ! -f "${ardupilot_pic_stamp}" \
            || "$(<"${ardupilot_pic_stamp}")" != "linux-fpic-v1" ) ]]; then
        # A pre-stamp archive was built without -fPIC.  Removing only the
        # archive is insufficient because waf would reuse its non-PIC objects.
        cmake -E remove_directory "${ardupilot_root}/build/sitl"
        needs_configure=1
    fi
    if [[ "${needs_configure}" -eq 1 ]]; then
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
        -DHAKONIWA_AP_ENABLE_EKF3=ON
        "-DHAKO_ENABLE_SENSOR_MODEL_V2=${HAKO_ENABLE_SENSOR_MODEL_V2:-OFF}"
        # Same switch as the PX4 adapter's build.bash: the plugin must be built
        # against the same control adapter interface revision as Drone PRO.
        "-DHAKO_ENABLE_GOTO_SPEED_LIMIT=${HAKO_ENABLE_GOTO_SPEED_LIMIT:-OFF}")
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
