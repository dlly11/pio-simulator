# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.21)
project(pio_sim LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Target chip: RP2040 (PIO version 0) or RP2350 (PIO version 1, a superset).
# Maps to PIO_SIM_PIO_VERSION (see lib/config/pio_sim_config.h) and to the
# `pioasm -v <0|1>` flag used by the encoding differential test. The integer
# `PIO_SIM_PIO_VERSION` is exported to the parent scope so tests/ can reuse it.
set(PIO_SIM_PLATFORM "RP2350" CACHE STRING
    "Target PIO platform: RP2040 (PIO v0) or RP2350 (PIO v1)")
set_property(CACHE PIO_SIM_PLATFORM PROPERTY STRINGS RP2040 RP2350)
if(PIO_SIM_PLATFORM STREQUAL "RP2040")
    set(PIO_SIM_PIO_VERSION 0)
elseif(PIO_SIM_PLATFORM STREQUAL "RP2350")
    set(PIO_SIM_PIO_VERSION 1)
else()
    message(FATAL_ERROR "PIO_SIM_PLATFORM must be RP2040 or RP2350 (got '${PIO_SIM_PLATFORM}')")
endif()

# The simulator + assembler. Pure C, no dependencies. Consumers link `pio_sim`
# and get the public headers (pio_sim.h, pio_asm.h) transitively.
add_library(pio_sim STATIC lib/src/pio_sim.c lib/src/pio_asm.c)
target_include_directories(pio_sim
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/config
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/src)

# PUBLIC: the public headers gate their enum/prototype surface on this macro, so
# consumers must compile against the same value the library was built with.
target_compile_definitions(pio_sim PUBLIC PIO_SIM_PIO_VERSION=${PIO_SIM_PIO_VERSION})

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pio_sim PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
        -Wcast-qual -Wcast-align -Wundef -Wwrite-strings -Wstrict-prototypes
        -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wswitch-enum)
    # Treat warnings as errors only for a standalone (top-level) build, so
    # embedders aren't broken by a different compiler's warning set.
    if(PROJECT_IS_TOP_LEVEL)
        target_compile_options(pio_sim PRIVATE -Werror)
    endif()
endif()

option(PIO_SIM_BUILD_TESTS "Build the pio_sim unit tests" ${PROJECT_IS_TOP_LEVEL})
if(PIO_SIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
