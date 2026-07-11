# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.21)
project(pio_sim VERSION 0.1.1 LANGUAGES C)

include(GNUInstallDirs)

# The library is written to C11 (what the headers and README promise); keep the
# build floor there so embedders on older toolchains aren't forced past it.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
# Build as strict ISO C11 (-std=c11), not the -std=gnu11 default, so a GNU-only
# extension is caught here rather than by a stricter downstream/MSVC toolchain —
# the "pure C11" invariant the README and CHANGELOG advertise.
set(CMAKE_C_EXTENSIONS OFF)

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

# Source-based coverage (llvm-cov). Instruments every target; the report is
# filtered to lib/src. Applied before the targets so it reaches all of them.
option(PIO_SIM_COVERAGE "Instrument for llvm-cov source coverage (Clang)" OFF)
if(PIO_SIM_COVERAGE)
    add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
    add_link_options(-fprofile-instr-generate -fcoverage-mapping)
endif()

# The simulator + assembler. Pure C, no dependencies. Consumers link `pio_sim`
# and get the public headers (pio_sim.h, pio_asm.h) transitively.
add_library(pio_sim STATIC lib/src/pio_sim.c lib/src/pio_gpio.c lib/src/pio_clk.c
    lib/src/pio_dma.c lib/src/pio_chip.c lib/src/pio_asm.c)
target_include_directories(pio_sim
    PUBLIC
        # BUILD_INTERFACE for in-tree/add_subdirectory use; INSTALL_INTERFACE so
        # an installed + find_package'd consumer resolves the same headers.
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/config>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/src)

# PUBLIC: the public headers gate their enum/prototype surface on this macro, so
# consumers must compile against the same value the library was built with.
target_compile_definitions(pio_sim PUBLIC PIO_SIM_PIO_VERSION=${PIO_SIM_PIO_VERSION})

# Shared warning set. Reused by the examples (examples/CMakeLists.txt) so the
# public-API reference code is held to the same standard as the library itself.
set(PIO_SIM_WARNING_FLAGS
    -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
    -Wcast-qual -Wcast-align -Wundef -Wwrite-strings -Wstrict-prototypes
    -Wmissing-prototypes -Wdouble-promotion -Wformat=2 -Wswitch-enum)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pio_sim PRIVATE ${PIO_SIM_WARNING_FLAGS})
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

# Reference example programs (examples/). Off by default so vendored/embedder
# builds stay lean; CI turns it on to build and run them against the live API.
option(PIO_SIM_BUILD_EXAMPLES "Build the reference example programs" OFF)
if(PIO_SIM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ── Documentation (Doxygen) ──────────────────────────────────────────────────
# `cmake --build build --target docs` renders the public-API HTML reference from
# the checked-in Doxyfile into docs/html (the same Doxyfile the Pages workflow
# uses, so local and published docs match). The target is only added when
# Doxygen is installed; a build without it just doesn't offer `docs`.
option(PIO_SIM_BUILD_DOCS "Add the 'docs' Doxygen target when Doxygen is found" ${PROJECT_IS_TOP_LEVEL})
if(PIO_SIM_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation (Doxygen) into docs/html"
            VERBATIM)
    else()
        message(STATUS "Doxygen not found — the 'docs' target is unavailable")
    endif()
endif()

# ── Install / packaging ──────────────────────────────────────────────────────
# Lets downstreams `find_package(pio_sim CONFIG)` and link pio_sim::pio_sim,
# not just add_subdirectory(). The PIO_SIM_PIO_VERSION compile definition is
# PUBLIC, so a consumer of the installed library gets the same platform value
# the library was built with. Opt-out for vendored/add_subdirectory embedders.
option(PIO_SIM_INSTALL "Generate install and packaging rules" ${PROJECT_IS_TOP_LEVEL})
if(PIO_SIM_INSTALL)
    include(CMakePackageConfigHelpers)

    install(TARGETS pio_sim EXPORT pio_simTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(DIRECTORY lib/include/ lib/config/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")

    install(EXPORT pio_simTargets
        FILE pio_simTargets.cmake
        NAMESPACE pio_sim::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pio_sim)

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pio_simConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/pio_simConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pio_sim)
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/pio_simConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/pio_simConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/pio_simConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pio_sim)
endif()
