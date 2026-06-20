# SPDX-License-Identifier: MIT
# Unit tests for the PIO simulator and assembler, on a vendored Unity.

add_library(unity STATIC unity/unity.c)
target_include_directories(unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/unity)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
target_compile_options(unity PRIVATE -w)

foreach(t test_pio_sim test_pio_asm)
    add_executable(${t} ${t}.c)
    target_link_libraries(${t} PRIVATE pio_sim unity)
    add_test(NAME ${t} COMMAND ${t})
endforeach()
