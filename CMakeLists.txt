cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_ap LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

set(ADAPTER_INTERFACE_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hakoniwa-drone-control-adapter")
set(ARDUPILOT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ardupilot")
set(ARDUPILOT_BUILD_ROOT "${ARDUPILOT_ROOT}/build/sitl")
if(NOT EXISTS "${ARDUPILOT_BUILD_ROOT}/ap_config.h")
    message(FATAL_ERROR
        "ArduPilot SITL configuration is missing: ${ARDUPILOT_BUILD_ROOT}/ap_config.h\n"
        "Run 'bash build.bash ardupilot' first.")
endif()

add_library(hakoniwa_drone_control_adapter_ap STATIC
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

if(BUILD_TESTING)
    add_executable(ap_rate_control_backend_smoke
        test/ap_rate_control_backend_smoke.cpp)
    target_link_libraries(ap_rate_control_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    add_test(NAME ap_rate_control_backend_smoke
        COMMAND ap_rate_control_backend_smoke)
endif()
