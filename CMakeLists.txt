cmake_minimum_required(VERSION 3.21)

# ws-hal-stm32f7-embedded-c — STM32F7 HAL base library (rcc, gpio, timebase, uart, spi).
project(ws-hal-stm32f7-embedded-c
    VERSION 0.2.0
    DESCRIPTION "ws HAL base library for STM32F7: rcc + gpio + timebase + uart + spi"
    LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set(WS_HAL_WARNING_FLAGS -Wall -Wextra -Werror)
else()
    set(WS_HAL_WARNING_FLAGS "")
endif()

if(CMAKE_CROSSCOMPILING)
    set(_ws_hal_fake_default OFF)
else()
    set(_ws_hal_fake_default ON)
endif()

option(WS_HAL_STM32F7_USE_FAKE
    "Build the host fake implementation instead of the CMSIS register implementation"
    ${_ws_hal_fake_default})

set(CMSIS_INCLUDE_DIRS "" CACHE STRING
    "Semicolon-separated CMSIS include dirs (cmsis_device_f7/Include;CMSIS_6/CMSIS/Core/Include)")

# Resolve ws-error-embedded-c without reaching into a parent project tree.
if(NOT TARGET ws-error-embedded-c)
    set(WS_ERROR_DIR "" CACHE PATH "Path to ws-error-embedded-c checkout (or resolved from $WS_ROOT/ws-libs)")
    set(_ws_err_src "${WS_ERROR_DIR}")
    if(NOT _ws_err_src AND DEFINED ENV{WS_ROOT} AND EXISTS "$ENV{WS_ROOT}/ws-libs/ws-error-embedded-c/CMakeLists.txt")
        set(_ws_err_src "$ENV{WS_ROOT}/ws-libs/ws-error-embedded-c")
    endif()
    if(_ws_err_src AND EXISTS "${_ws_err_src}/CMakeLists.txt")
        add_subdirectory("${_ws_err_src}" "${CMAKE_CURRENT_BINARY_DIR}/_deps/ws-error-embedded-c")
    else()
        find_package(ws-error-embedded-c REQUIRED)
    endif()
endif()

# Resolve CMSIS headers from CMSIS_INCLUDE_DIRS or $WS_ROOT/ws-libs/ (never ../../../).
set(_ws_cmsis_dirs ${CMSIS_INCLUDE_DIRS})
if(NOT _ws_cmsis_dirs AND DEFINED ENV{WS_ROOT})
    foreach(_probe
            "$ENV{WS_ROOT}/ws-libs/cmsis_device_f7/Include"
            "$ENV{WS_ROOT}/ws-libs/CMSIS_6/CMSIS/Core/Include")
        if(IS_DIRECTORY "${_probe}")
            list(APPEND _ws_cmsis_dirs "${_probe}")
        endif()
    endforeach()
endif()

set(_ws_cmsis_valid FALSE)
set(_ws_cmsis_missing "")
if(_ws_cmsis_dirs)
    set(_ws_cmsis_valid TRUE)
    foreach(_dir IN LISTS _ws_cmsis_dirs)
        if(NOT IS_DIRECTORY "${_dir}")
            set(_ws_cmsis_valid FALSE)
            list(APPEND _ws_cmsis_missing "${_dir}")
        endif()
    endforeach()
endif()

if(WS_HAL_STM32F7_USE_FAKE)
    add_library(ws-hal-stm32f7-embedded-c STATIC
        fake/rcc_fake.c
        fake/gpio_fake.c
        fake/timebase_fake.c
        fake/uart_fake.c
        fake/spi_fake.c)
    target_compile_definitions(ws-hal-stm32f7-embedded-c PUBLIC WS_HAL_STM32F7_HOST_FAKE)
else()
    if(NOT _ws_cmsis_valid)
        message(FATAL_ERROR
            "ws-hal-stm32f7-embedded-c: WS_HAL_STM32F7_USE_FAKE=OFF requires existing CMSIS include "
            "directories via -DCMSIS_INCLUDE_DIRS=... or synced $WS_ROOT/ws-libs/{cmsis_device_f7,CMSIS_6}. "
            "Missing: '${_ws_cmsis_missing}' (configured: '${_ws_cmsis_dirs}').")
    endif()
    add_library(ws-hal-stm32f7-embedded-c STATIC
        src/rcc.c
        src/gpio.c
        src/timebase.c
        src/uart.c
        src/spi.c)
    target_compile_definitions(ws-hal-stm32f7-embedded-c PRIVATE STM32F746xx)
    target_include_directories(ws-hal-stm32f7-embedded-c PRIVATE ${_ws_cmsis_dirs})
endif()

add_library(ws-hal::ws-hal-stm32f7-embedded-c ALIAS ws-hal-stm32f7-embedded-c)
target_link_libraries(ws-hal-stm32f7-embedded-c PUBLIC ws-error-embedded-c)
target_include_directories(ws-hal-stm32f7-embedded-c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_options(ws-hal-stm32f7-embedded-c PRIVATE ${WS_HAL_WARNING_FLAGS})

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND WS_HAL_STM32F7_USE_FAKE)
    include(CTest)
    if(BUILD_TESTING)
        foreach(_unit rcc gpio timebase uart spi)
            add_executable(${_unit}_test tests/${_unit}_test.c)
            target_link_libraries(${_unit}_test PRIVATE ws-hal-stm32f7-embedded-c)
            target_compile_options(${_unit}_test PRIVATE ${WS_HAL_WARNING_FLAGS})
            add_test(NAME ${_unit}_test COMMAND ${_unit}_test)
        endforeach()
        foreach(_ex rcc_clocks gpio_blink timebase_delay uart_hello spi_transfer)
            add_executable(${_ex}_example examples/${_ex}.c)
            target_link_libraries(${_ex}_example PRIVATE ws-hal-stm32f7-embedded-c)
            target_compile_options(${_ex}_example PRIVATE ${WS_HAL_WARNING_FLAGS})
        endforeach()
    endif()

    # Optional target cross-check (single-precision FPv5-SP-D16 per DS10916 Rev 5).
    find_program(WS_ARM_NONE_EABI_GCC NAMES arm-none-eabi-gcc)
    if(WS_ARM_NONE_EABI_GCC AND _ws_cmsis_valid)
        set(_ws_cross_out "${CMAKE_CURRENT_BINARY_DIR}/cross-check")
        set(_ws_cross_incs)
        foreach(_dir IN LISTS _ws_cmsis_dirs)
            list(APPEND _ws_cross_incs "-I${_dir}")
        endforeach()
        get_target_property(_ws_err_incs ws-error-embedded-c INTERFACE_INCLUDE_DIRECTORIES)
        foreach(_dir IN LISTS _ws_err_incs)
            if(NOT _dir MATCHES "^\\$<INSTALL_INTERFACE:")
                string(REGEX REPLACE "^\\$<BUILD_INTERFACE:(.*)>$" "\\1" _clean_dir "${_dir}")
                list(APPEND _ws_cross_incs "-I${_clean_dir}")
            endif()
        endforeach()

        set(_ws_cross_objs)
        foreach(_src rcc gpio timebase uart spi)
            set(_obj "${_ws_cross_out}/${_src}.o")
            add_custom_command(
                OUTPUT "${_obj}"
                COMMAND ${CMAKE_COMMAND} -E make_directory "${_ws_cross_out}"
                COMMAND ${WS_ARM_NONE_EABI_GCC}
                    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
                    -std=c17 -Wall -Wextra -Werror -DSTM32F746xx
                    ${_ws_cross_incs}
                    "-I${CMAKE_CURRENT_SOURCE_DIR}/include"
                    -c "${CMAKE_CURRENT_SOURCE_DIR}/src/${_src}.c"
                    -o "${_obj}"
                DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/${_src}.c"
                        "${CMAKE_CURRENT_SOURCE_DIR}/include/ws/hal/stm32f7/rcc.h"
                        "${CMAKE_CURRENT_SOURCE_DIR}/include/ws/hal/stm32f7/gpio.h"
                        "${CMAKE_CURRENT_SOURCE_DIR}/include/ws/hal/stm32f7/timebase.h"
                        "${CMAKE_CURRENT_SOURCE_DIR}/include/ws/hal/stm32f7/uart.h"
                        "${CMAKE_CURRENT_SOURCE_DIR}/include/ws/hal/stm32f7/spi.h"
                COMMENT "ws-hal-stm32f7 cross-check: src/${_src}.c (Cortex-M7 fpv5-sp-d16)"
                VERBATIM)
            list(APPEND _ws_cross_objs "${_obj}")
        endforeach()
        add_custom_target(ws-hal-stm32f7-cross-check ALL DEPENDS ${_ws_cross_objs})
    endif()
endif()
