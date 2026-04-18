cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Project: rl_host
# Description:
#   Windows host controller for the RISC-V Architectural Differential Fuzzing
#   Lab. Supports serial communication with the FPGA/boards, Rigol TCP/SCPI
#   scope capture, waveform analysis, logging, and reinforcement-learning based
#   test scheduling.
#
# Generator:
#   Intended for Visual Studio 17 2022 multi-config builds.
#
# Example:
#   cmake -S . -B build -G "Visual Studio 17 2022"
#   cmake --build build --config Debug
#   cmake --build build --config Release
# =============================================================================
project(rl_host LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# =============================================================================
# Output directories
#
# For multi-config generators such as Visual Studio, binaries are placed in:
#   build/Debug/
#   build/Release/
# This keeps configurations separated within one generated build tree.
# =============================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${cfg}" cfg_upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg_upper} ${CMAKE_BINARY_DIR}/${cfg})
endforeach()

# =============================================================================
# Source files
# =============================================================================
set(RL_HOST_SOURCES
    main.c
    serial.c
    rigol.c
    waveform.c
    rl.c
    logging.c
)

set(RL_HOST_HEADERS
    common.h
    serial.h
    rigol.h
    waveform.h
    rl.h
)

# =============================================================================
# Executable target
# =============================================================================
add_executable(rl_host ${RL_HOST_SOURCES} ${RL_HOST_HEADERS})

target_include_directories(rl_host PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Organize files nicely in Visual Studio Solution Explorer.
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${RL_HOST_SOURCES} ${RL_HOST_HEADERS})

# =============================================================================
# Compiler warnings and config-specific options
# =============================================================================
if(MSVC)
    target_compile_options(rl_host PRIVATE
        /W4
        /permissive-
        $<$<CONFIG:Debug>:/Od /Zi>
        $<$<CONFIG:Release>:/O2>
    )
    target_link_options(rl_host PRIVATE
        $<$<CONFIG:Debug>:/DEBUG>
    )
else()
    target_compile_options(rl_host PRIVATE
        -Wall -Wextra -Wpedantic
        $<$<CONFIG:Debug>:-O0 -g>
        $<$<CONFIG:Release>:-O2>
    )
endif()

# =============================================================================
# Platform-specific libraries
# =============================================================================
if(WIN32)
    target_link_libraries(rl_host PRIVATE ws2_32)
endif()

# Math library is needed on many non-MSVC toolchains.
if(NOT MSVC)
    target_link_libraries(rl_host PRIVATE m)
endif()

# =============================================================================
# IDE convenience
# =============================================================================
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT rl_host)
