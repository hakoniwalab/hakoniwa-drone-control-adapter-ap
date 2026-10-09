cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_ap LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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
    src/ap_allocation_feedback_policy.cpp
    src/ap_altitude_control_backend.cpp
    src/ap_attitude_control_backend.cpp
    src/ap_control_allocation_backend.cpp
    src/ap_horizontal_position_control_backend.cpp
    src/ap_rate_control_backend.cpp
    src/ardupilot_controller_config_loader.cpp
    src/ardupilot_runtime_shim.cpp
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_P_1D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_P_2D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_PID.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_PID_2D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AP_Math/vector2.cpp"
    "${ARDUPILOT_ROOT}/libraries/Filter/LowPassFilter.cpp"
    "${ARDUPILOT_ROOT}/libraries/Filter/SlewCalculator2D.cpp"
    "${ARDUPILOT_ROOT}/libraries/Filter/SlewLimiter.cpp")

add_library(
    hakoniwa_drone_control_adapter_ap::hakoniwa_drone_control_adapter_ap
    ALIAS hakoniwa_drone_control_adapter_ap)
# The plugin (below) links this archive into a shared library.
set_target_properties(hakoniwa_drone_control_adapter_ap PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(hakoniwa_drone_control_adapter_ap
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${ADAPTER_INTERFACE_ROOT}/include>
        $<INSTALL_INTERFACE:include>
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
    add_library(ardupilot_dal STATIC IMPORTED GLOBAL)
    set_target_properties(ardupilot_dal PROPERTIES
        IMPORTED_LOCATION "${ARDUPILOT_DAL_LIBRARY}")
    add_library(hakoniwa_drone_control_adapter_ap_ekf3 STATIC
        src/ap_ekf3_backend.cpp
        src/ardupilot_ekf3_runtime_shim.cpp)
    add_library(
        hakoniwa_drone_control_adapter_ap::ekf3
        ALIAS hakoniwa_drone_control_adapter_ap_ekf3)
    target_include_directories(hakoniwa_drone_control_adapter_ap_ekf3
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${ADAPTER_INTERFACE_ROOT}/include>
            $<INSTALL_INTERFACE:include>
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
        PRIVATE ardupilot_dal)
endif()

# The Control Link plugin (hakoniwa-drone-control-adapter adapter_plugin.hpp): a
# host loads it at run time (controller.backendType adapter-plugin).
add_library(hakoniwa_drone_control_adapter_ap_plugin MODULE plugin/ap_control_adapter_plugin.cpp)
target_link_libraries(hakoniwa_drone_control_adapter_ap_plugin PRIVATE hakoniwa_drone_control_adapter_ap)
target_compile_definitions(hakoniwa_drone_control_adapter_ap_plugin PRIVATE
    HAKO_AP_CAPABILITY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/contract/capabilities/ardupilot.json")
set_target_properties(hakoniwa_drone_control_adapter_ap_plugin PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
install(TARGETS hakoniwa_drone_control_adapter_ap_plugin LIBRARY DESTINATION lib)

include(CMakePackageConfigHelpers)
install(TARGETS hakoniwa_drone_control_adapter_ap
    EXPORT hakoniwa_drone_control_adapter_apTargets ARCHIVE DESTINATION lib)
if(HAKONIWA_AP_ENABLE_EKF3)
    install(TARGETS hakoniwa_drone_control_adapter_ap_ekf3
        EXPORT hakoniwa_drone_control_adapter_apTargets ARCHIVE DESTINATION lib)
    install(FILES "${ARDUPILOT_DAL_LIBRARY}" DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY thirdparty/hakoniwa-drone-control-adapter/include/ DESTINATION include)
install(EXPORT hakoniwa_drone_control_adapter_apTargets
    NAMESPACE hakoniwa_drone_control_adapter_ap::
    DESTINATION lib/cmake/hakoniwa_drone_control_adapter_ap)
configure_package_config_file(cmake/hakoniwa_drone_control_adapter_apConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_apConfig.cmake
    INSTALL_DESTINATION lib/cmake/hakoniwa_drone_control_adapter_ap)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_apConfigVersion.cmake
    VERSION 1.0.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_apConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/hakoniwa_drone_control_adapter_apConfigVersion.cmake
    DESTINATION lib/cmake/hakoniwa_drone_control_adapter_ap)

if(BUILD_TESTING)
    add_executable(ap_contract_smoke test/ap_contract_smoke.cpp)
    target_link_libraries(ap_contract_smoke PRIVATE hakoniwa_drone_control_adapter_ap)
    add_test(NAME ap_contract_smoke COMMAND ap_contract_smoke
        ${CMAKE_CURRENT_SOURCE_DIR}/config/ardupilot-controller-config.json)
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
