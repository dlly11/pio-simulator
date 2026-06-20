# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.21)
project(pio_sim LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# The simulator + assembler. Pure C, no dependencies. Consumers link `pio_sim`
# and get the public headers (pio_sim.h, pio_asm.h) transitively.
add_library(pio_sim STATIC pio_sim.c pio_asm.c)
target_include_directories(pio_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pio_sim PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
        -Wcast-qual -Wcast-align -Wundef -Wwrite-strings -Wstrict-prototypes
        -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wswitch-enum)
    # Treat warnings as errors only for a standalone (top-level) build, so
    # embedders (Aperix, dispatch) aren't broken by a different compiler's
    # warning set.
    if(PROJECT_IS_TOP_LEVEL)
        target_compile_options(pio_sim PRIVATE -Werror)
    endif()
endif()

option(PIO_SIM_BUILD_TESTS "Build the pio_sim unit tests" ${PROJECT_IS_TOP_LEVEL})
if(PIO_SIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
