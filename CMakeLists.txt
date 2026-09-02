cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_ap LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
option(HAKONIWA_AP_ENABLE_EKF3
    "Build the standalone ArduPilot NavEKF3 backend" OFF)

set(ADAPTER_INTERFACE_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hakoniwa-drone-control-adapter")
set(ARDUPILOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ardupilot")
set(ARDUPILOT_BUILD_ROOT "${ARDUPILOT_ROOT}/build/sitl")
set(ARDUPILOT_DAL_LIBRARY "${ARDUPILOT_BUILD_ROOT}/lib/libAP_DAL_libs.a")
if(NOT EXISTS "${ARDUPILOT_BUILD_ROOT}/ap_config.h")
    message(FATAL_ERROR
        "ArduPilot SITL configuration is missing: ${ARDUPILOT_BUILD_ROOT}/ap_config.h\n"
        "Run 'bash build.bash ardupilot' first.")
endif()

add_library(hakoniwa_drone_control_adapter_ap STATIC
    src/ap_attitude_control_backend.cpp
    src/ap_control_allocation_backend.cpp
    src/ap_rate_control_backend.cpp
    src/ardupilot_runtime_shim.cpp
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_PID.cpp"
    "${ARDUPILOT_ROOT}/libraries/AP_Math/vector2.cpp"
    "${ARDUPILOT_ROOT}/libraries/Filter/LowPassFilter.cpp"
    "${ARDUPILOT_ROOT}/libraries/Filter/SlewLimiter.cpp")

add_library(
    hakoniwa_drone_control_adapter_ap::hakoniwa_drone_control_adapter_ap
    ALIAS hakoniwa_drone_control_adapter_ap)

target_include_directories(hakoniwa_drone_control_adapter_ap
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${ADAPTER_INTERFACE_ROOT}/include"
    PRIVATE
        "${ARDUPILOT_BUILD_ROOT}"
        "${ARDUPILOT_ROOT}/libraries")

target_compile_options(hakoniwa_drone_control_adapter_ap PRIVATE
    "SHELL:-include ap_config.h"
    "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/include/hakoniwa/drone/control_adapter/ardupilot_adapter_config.hpp"
    "-Wno-expansion-to-defined")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(hakoniwa_drone_control_adapter_ap PRIVATE
        "-Wno-gnu-designator"
        "-Wno-macro-redefined")
endif()

target_compile_definitions(hakoniwa_drone_control_adapter_ap PRIVATE
    ARDUPILOT_BUILD
    AP_FILTER_NUM_FILTERS=0
    AP_INLINE_VECTOR_OPS
    __AP_LINE__=__LINE__
    CONFIG_HAL_BOARD=HAL_BOARD_SITL
    CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE)

if(HAKONIWA_AP_ENABLE_EKF3 AND NOT EXISTS "${ARDUPILOT_DAL_LIBRARY}")
    message(FATAL_ERROR
        "ArduPilot DAL library is missing: ${ARDUPILOT_DAL_LIBRARY}\n"
        "Run 'bash build.bash ardupilot' first.")
endif()

if(HAKONIWA_AP_ENABLE_EKF3)
    add_library(hakoniwa_drone_control_adapter_ap_ekf3 STATIC
        src/ap_ekf3_backend.cpp
        src/ardupilot_ekf3_runtime_shim.cpp)
    add_library(
        hakoniwa_drone_control_adapter_ap::ekf3
        ALIAS hakoniwa_drone_control_adapter_ap_ekf3)
    target_include_directories(hakoniwa_drone_control_adapter_ap_ekf3
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${ADAPTER_INTERFACE_ROOT}/include"
        PRIVATE
            "${ARDUPILOT_BUILD_ROOT}"
            "${ARDUPILOT_BUILD_ROOT}/libraries/GCS_MAVLink"
            "${ARDUPILOT_ROOT}/libraries"
            "${ARDUPILOT_ROOT}/libraries/GCS_MAVLink"
            "${ARDUPILOT_ROOT}/modules/mavlink")
    target_compile_options(hakoniwa_drone_control_adapter_ap_ekf3 PRIVATE
        "SHELL:-include ap_config.h"
        "-Wno-expansion-to-defined"
        "-Wno-trigraphs")
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(hakoniwa_drone_control_adapter_ap_ekf3 PRIVATE
            "-Wno-gnu-designator"
            "-Wno-macro-redefined")
    endif()
    target_compile_definitions(hakoniwa_drone_control_adapter_ap_ekf3 PRIVATE
        ARDUPILOT_BUILD
        __AP_LINE__=__LINE__
        CONFIG_HAL_BOARD=HAL_BOARD_SITL
        CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE
        HAL_GCS_ENABLED=0
        HAL_NAVEKF3_AVAILABLE=1
        APM_BUILD_DIRECTORY=APM_BUILD_AP_DAL_Standalone
        HAKONIWA_AP_DAL_ZERO_IMU_OFFSET=1
        HAL_EKF_IMU_MASK_DEFAULT=1)
    target_link_libraries(hakoniwa_drone_control_adapter_ap_ekf3
        PRIVATE "${ARDUPILOT_DAL_LIBRARY}")
endif()

if(BUILD_TESTING)
    add_executable(ap_attitude_control_backend_smoke
        test/ap_attitude_control_backend_smoke.cpp)
    target_link_libraries(ap_attitude_control_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    add_test(NAME ap_attitude_control_backend_smoke
        COMMAND ap_attitude_control_backend_smoke)

    add_executable(ap_control_allocation_backend_smoke
        test/ap_control_allocation_backend_smoke.cpp)
    target_link_libraries(ap_control_allocation_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    add_test(NAME ap_control_allocation_backend_smoke
        COMMAND ap_control_allocation_backend_smoke)

    add_executable(ap_rate_control_backend_smoke
        test/ap_rate_control_backend_smoke.cpp)
    target_link_libraries(ap_rate_control_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    add_test(NAME ap_rate_control_backend_smoke
        COMMAND ap_rate_control_backend_smoke)

    if(HAKONIWA_AP_ENABLE_EKF3)
        add_executable(ap_ekf3_backend_smoke
            test/ap_ekf3_backend_smoke.cpp)
        target_link_libraries(ap_ekf3_backend_smoke
            PRIVATE hakoniwa_drone_control_adapter_ap_ekf3)
        if(APPLE)
            target_link_options(ap_ekf3_backend_smoke PRIVATE "-Wl,-dead_strip")
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_link_options(ap_ekf3_backend_smoke PRIVATE "-Wl,--gc-sections")
        endif()
        add_test(NAME ap_ekf3_backend_smoke
            COMMAND ap_ekf3_backend_smoke)

        add_executable(ap_ekf3_backend_motion_smoke
            test/ap_ekf3_backend_motion_smoke.cpp)
        target_link_libraries(ap_ekf3_backend_motion_smoke
            PRIVATE hakoniwa_drone_control_adapter_ap_ekf3)
        if(APPLE)
            target_link_options(ap_ekf3_backend_motion_smoke PRIVATE "-Wl,-dead_strip")
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_link_options(ap_ekf3_backend_motion_smoke PRIVATE "-Wl,--gc-sections")
        endif()
        add_test(NAME ap_ekf3_backend_motion_smoke
            COMMAND ap_ekf3_backend_motion_smoke)
    endif()
endif()
