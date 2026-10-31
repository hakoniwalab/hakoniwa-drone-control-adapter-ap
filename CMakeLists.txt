cmake_minimum_required(VERSION 3.16)

project(hakoniwa_drone_control_adapter_ap LANGUAGES CXX)

option(HAKO_ENABLE_GOTO_SPEED_LIMIT
    "Build against the goto-speed-enabled control adapter interface" OFF)
if(HAKO_ENABLE_GOTO_SPEED_LIMIT)
    add_compile_definitions(HAKO_ENABLE_GOTO_SPEED_LIMIT)
endif()

option(HAKO_ENABLE_SENSOR_MODEL_V2
    "Apply ArduPilot sensor orientation, lever arms, and compass calibration" OFF)
if(HAKO_ENABLE_SENSOR_MODEL_V2)
    add_compile_definitions(HAKO_ENABLE_SENSOR_MODEL_V2)
endif()

# ArduPilot's plugin always implements the flight-phase contract.  The public
# headers, static library, plugin and tests must use the same ABI revision.
add_compile_definitions(HAKO_ENABLE_FLIGHT_PHASE)

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
set(ARDUPILOT_DAL_FILTERED_LIBRARY
    "${CMAKE_CURRENT_BINARY_DIR}/libAP_DAL_libs_hakoniwa.a")
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
    src/ap_position_control_3d_backend.cpp
    src/ap_rate_control_backend.cpp
    src/ap_sensor_model_v2.cpp
    src/ardupilot_controller_config_loader.cpp
    src/ardupilot_runtime_shim.cpp
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_P_1D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_P_2D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_PID.cpp"
    "${ARDUPILOT_ROOT}/libraries/AC_PID/AC_PID_2D.cpp"
    "${ARDUPILOT_ROOT}/libraries/AP_Math/control.cpp"
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
    __AP_LINE__=__LINE__
    CONFIG_HAL_BOARD=HAL_BOARD_SITL
    CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE)
if(HAKO_ENABLE_SENSOR_MODEL_V2)
    # Custom 1 is built from its configured Euler angles below; standard
    # Vector3::rotate must not pull in ArduPilot's AP_Param singleton graph.
    target_compile_definitions(hakoniwa_drone_control_adapter_ap PRIVATE AP_CUSTOMROTATIONS_ENABLED=0)
    target_sources(hakoniwa_drone_control_adapter_ap PRIVATE
        "${ARDUPILOT_ROOT}/libraries/AP_Math/AP_Math.cpp"
        "${ARDUPILOT_ROOT}/libraries/AP_Math/matrix3.cpp"
        "${ARDUPILOT_ROOT}/libraries/AP_Math/vector3.cpp")
else()
    target_compile_definitions(hakoniwa_drone_control_adapter_ap PRIVATE AP_INLINE_VECTOR_OPS)
endif()

if(HAKONIWA_AP_ENABLE_EKF3 AND NOT EXISTS "${ARDUPILOT_DAL_LIBRARY}")
    message(FATAL_ERROR
        "ArduPilot DAL library is missing: ${ARDUPILOT_DAL_LIBRARY}\n"
        "Run 'bash build.bash ardupilot' first.")
endif()

if(HAKONIWA_AP_ENABLE_EKF3)
    # AP_HAL_SITL/system.cpp.o assumes a fully initialized SITL HAL, whereas
    # this plugin supplies its own standalone clock.  AP_Math.cpp.o remains in
    # the DAL archive and supplies calc_lowpass_alpha_dt() for this EKF build.
    # Make one archive without that member on every platform, so the system
    # API has no second definition at the final link.
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${ARDUPILOT_DAL_LIBRARY}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy
            "${ARDUPILOT_DAL_LIBRARY}" "${ARDUPILOT_DAL_FILTERED_LIBRARY}"
        COMMAND_ERROR_IS_FATAL ANY)
    execute_process(
        COMMAND "${CMAKE_AR}" d "${ARDUPILOT_DAL_FILTERED_LIBRARY}"
            system.cpp.0.o
        COMMAND_ERROR_IS_FATAL ANY)
    if(HAKO_ENABLE_SENSOR_MODEL_V2)
        execute_process(
            COMMAND "${CMAKE_AR}" d "${ARDUPILOT_DAL_FILTERED_LIBRARY}"
                AP_Math.cpp.0.o vector3.cpp.0.o matrix3.cpp.0.o
            COMMAND_ERROR_IS_FATAL ANY)
    endif()
    add_library(ardupilot_dal STATIC IMPORTED GLOBAL)
    set_target_properties(ardupilot_dal PROPERTIES
        IMPORTED_LOCATION "${ARDUPILOT_DAL_FILTERED_LIBRARY}")
    # The standalone smoke executables do not load DAL. Give those executables
    # their own one-symbol helper; the plugin does not link this target and
    # gets the sole EKF-build definition from DAL's AP_Math.cpp.o instead.
    add_library(hakoniwa_drone_control_adapter_ap_lowpass STATIC
        src/ardupilot_lowpass_helper.cpp)
    target_include_directories(hakoniwa_drone_control_adapter_ap_lowpass PRIVATE
        "${ARDUPILOT_BUILD_ROOT}"
        "${ARDUPILOT_ROOT}/libraries")
    target_compile_options(hakoniwa_drone_control_adapter_ap_lowpass PRIVATE
        "SHELL:-include ap_config.h"
        "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/include/hakoniwa/drone/control_adapter/ardupilot_adapter_config.hpp"
        "-Wno-expansion-to-defined")
    target_compile_definitions(hakoniwa_drone_control_adapter_ap_lowpass PRIVATE
        ARDUPILOT_BUILD
        AP_FILTER_NUM_FILTERS=0
        AP_INLINE_VECTOR_OPS
        __AP_LINE__=__LINE__
        CONFIG_HAL_BOARD=HAL_BOARD_SITL
        CONFIG_HAL_BOARD_SUBTYPE=HAL_BOARD_SUBTYPE_NONE)
    if(HAKO_ENABLE_SENSOR_MODEL_V2)
        target_compile_definitions(hakoniwa_drone_control_adapter_ap_lowpass PRIVATE
            HAKO_ENABLE_SENSOR_MODEL_V2)
    endif()
    function(hakoniwa_ap_link_standalone_lowpass target)
        target_link_libraries(${target} PRIVATE
            hakoniwa_drone_control_adapter_ap_lowpass)
    endfunction()
    add_library(hakoniwa_drone_control_adapter_ap_ekf3 STATIC
        src/ap_ekf3_backend.cpp
        src/ap_baro_atmosphere.cpp
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
    # AP_Math.cpp.o in the filtered DAL archive is the sole low-pass helper
    # provider in an EKF build.  Without EKF, the standalone shim provides it.
    target_compile_definitions(hakoniwa_drone_control_adapter_ap PRIVATE
        HAKONIWA_AP_DAL_MATH_LINKED=1)
    target_link_libraries(hakoniwa_drone_control_adapter_ap_ekf3
        PRIVATE ardupilot_dal hakoniwa_drone_control_adapter_ap)
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
if(HAKONIWA_AP_ENABLE_EKF3)
    # The plugin offers NavEKF3 as its EKF stage.
    set_target_properties(hakoniwa_drone_control_adapter_ap_ekf3 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(hakoniwa_drone_control_adapter_ap_plugin PRIVATE
        hakoniwa_drone_control_adapter_ap_ekf3 ardupilot_dal)
    target_compile_definitions(hakoniwa_drone_control_adapter_ap_plugin PRIVATE HAKO_AP_PLUGIN_WITH_EKF3)
    # As the EKF3 smoke tests: drop the DAL sections NavEKF3 does not reach.
    # Only the plugin entry is exported, so it is the one root to keep.
    if(APPLE)
        target_link_options(hakoniwa_drone_control_adapter_ap_plugin PRIVATE
            "-Wl,-dead_strip" "-Wl,-exported_symbol,_hako_control_adapter_plugin_v1")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ap_plugin_exports.map"
            "{ global: hako_control_adapter_plugin_v1; local: *; };\n")
        target_link_options(hakoniwa_drone_control_adapter_ap_plugin PRIVATE
            "-Wl,--gc-sections"
            "-Wl,--version-script=${CMAKE_CURRENT_BINARY_DIR}/ap_plugin_exports.map")
    else()
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ap_plugin_exports.map"
            "{ global: hako_control_adapter_plugin_v1; local: *; };\n")
        target_link_options(hakoniwa_drone_control_adapter_ap_plugin PRIVATE
            "-Wl,--gc-sections" "-Wl,--version-script=${CMAKE_CURRENT_BINARY_DIR}/ap_plugin_exports.map")
    endif()
endif()
install(TARGETS hakoniwa_drone_control_adapter_ap_plugin LIBRARY DESTINATION lib)

include(CMakePackageConfigHelpers)
install(TARGETS hakoniwa_drone_control_adapter_ap
    EXPORT hakoniwa_drone_control_adapter_apTargets ARCHIVE DESTINATION lib)
if(HAKONIWA_AP_ENABLE_EKF3)
    install(TARGETS hakoniwa_drone_control_adapter_ap_ekf3
        EXPORT hakoniwa_drone_control_adapter_apTargets ARCHIVE DESTINATION lib)
    install(FILES "${ARDUPILOT_DAL_FILTERED_LIBRARY}" DESTINATION lib
        RENAME "libAP_DAL_libs.a")
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
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME ardupilot_python_tools
        COMMAND ${Python3_EXECUTABLE} -m unittest discover
            -s ${CMAKE_CURRENT_SOURCE_DIR}/test -p test_*.py)

    if(HAKO_ENABLE_SENSOR_MODEL_V2)
        add_executable(ap_sensor_model_v2_smoke test/ap_sensor_model_v2_smoke.cpp)
        target_link_libraries(ap_sensor_model_v2_smoke PRIVATE hakoniwa_drone_control_adapter_ap)
        if(HAKONIWA_AP_ENABLE_EKF3)
            hakoniwa_ap_link_standalone_lowpass(ap_sensor_model_v2_smoke)
        endif()
        add_test(NAME ap_sensor_model_v2_smoke COMMAND ap_sensor_model_v2_smoke
            ${CMAKE_CURRENT_SOURCE_DIR}/config/ardupilot-controller-config.json)
    endif()

    add_executable(ap_contract_smoke test/ap_contract_smoke.cpp)
    target_link_libraries(ap_contract_smoke PRIVATE hakoniwa_drone_control_adapter_ap)
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_contract_smoke)
    endif()
    add_test(NAME ap_contract_smoke COMMAND ap_contract_smoke
        ${CMAKE_CURRENT_SOURCE_DIR}/config/ardupilot-controller-config.json)
    add_executable(ap_attitude_control_backend_smoke
        test/ap_attitude_control_backend_smoke.cpp)
    target_link_libraries(ap_attitude_control_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_attitude_control_backend_smoke)
    endif()
    add_test(NAME ap_attitude_control_backend_smoke
        COMMAND ap_attitude_control_backend_smoke)

    add_executable(ap_control_allocation_backend_smoke
        test/ap_control_allocation_backend_smoke.cpp)
    target_link_libraries(ap_control_allocation_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_control_allocation_backend_smoke)
    endif()
    add_test(NAME ap_control_allocation_backend_smoke
        COMMAND ap_control_allocation_backend_smoke)

    add_executable(ap_rate_control_backend_smoke
        test/ap_rate_control_backend_smoke.cpp)
    target_link_libraries(ap_rate_control_backend_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_rate_control_backend_smoke)
    endif()
    add_test(NAME ap_rate_control_backend_smoke
        COMMAND ap_rate_control_backend_smoke)

    add_executable(ap_position_control_backends_smoke
        test/ap_position_control_backends_smoke.cpp)
    target_link_libraries(ap_position_control_backends_smoke
        PRIVATE hakoniwa_drone_control_adapter_ap)
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_position_control_backends_smoke)
    endif()
    add_test(NAME ap_position_control_backends_smoke
        COMMAND ap_position_control_backends_smoke)

    add_executable(ap_sensor_filters_smoke test/ap_sensor_filters_smoke.cpp)
    target_include_directories(ap_sensor_filters_smoke PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/plugin)
    target_link_libraries(ap_sensor_filters_smoke PRIVATE
        hakoniwa_drone_control_adapter_ap ${CMAKE_DL_LIBS})
    if(HAKONIWA_AP_ENABLE_EKF3)
        hakoniwa_ap_link_standalone_lowpass(ap_sensor_filters_smoke)
    endif()
    add_dependencies(ap_sensor_filters_smoke hakoniwa_drone_control_adapter_ap_plugin)
    add_test(NAME ap_sensor_filters_smoke COMMAND ap_sensor_filters_smoke
        $<TARGET_FILE:hakoniwa_drone_control_adapter_ap_plugin>
        ${CMAKE_CURRENT_SOURCE_DIR}/config/ardupilot-controller-config.json)

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

        if(HAKO_ENABLE_SENSOR_MODEL_V2)
            add_executable(ap_sensor_model_v2_ekf_smoke test/ap_sensor_model_v2_ekf_smoke.cpp)
            target_link_libraries(ap_sensor_model_v2_ekf_smoke
                PRIVATE hakoniwa_drone_control_adapter_ap_ekf3 hakoniwa_drone_control_adapter_ap)
            if(APPLE)
                target_link_options(ap_sensor_model_v2_ekf_smoke PRIVATE "-Wl,-dead_strip")
            elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
                target_link_options(ap_sensor_model_v2_ekf_smoke PRIVATE "-Wl,--gc-sections")
            endif()
            add_test(NAME ap_sensor_model_v2_ekf_smoke COMMAND ap_sensor_model_v2_ekf_smoke)
        endif()
    endif()
endif()
