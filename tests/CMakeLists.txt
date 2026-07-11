# SPDX-License-Identifier: MIT
# Unit tests for the PIO simulator and assembler, on Unity (a git submodule
# under third_party/unity). Run `git submodule update --init` before configuring.

add_library(unity STATIC ${CMAKE_SOURCE_DIR}/third_party/unity/src/unity.c)
target_include_directories(unity PUBLIC
    ${CMAKE_SOURCE_DIR}/third_party/unity/src
    ${CMAKE_CURRENT_SOURCE_DIR})  # tests/ holds unity_config.h
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
target_compile_options(unity PRIVATE -w)

# Hold the test code to the library's full warning set (defined in the root
# CMakeLists) so a -Wconversion/-Wshadow regression in a test can't slip through.
# -Werror only on a standalone build, matching the library and examples; unity
# itself stays at -w above. Not applied to the fuzz harness: libFuzzer's entry
# point has no prototype, which trips -Wmissing-prototypes.
function(pio_sim_test_warnings target)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE ${PIO_SIM_WARNING_FLAGS})
        if(PROJECT_IS_TOP_LEVEL)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

foreach(t test_pio_sim test_pio_asm test_pio_gpio test_pio_clk test_pio_dma)
    add_executable(${t} ${t}.c)
    target_link_libraries(${t} PRIVATE pio_sim unity)
    pio_sim_test_warnings(${t})
    add_test(NAME ${t} COMMAND ${t})
endforeach()

# Real-program end-to-end tests read their .pio sources from the corpus dir.
add_executable(test_pio_programs test_pio_programs.c)
target_link_libraries(test_pio_programs PRIVATE pio_sim unity)
target_compile_definitions(test_pio_programs PRIVATE
    TESTS_PIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/pio")
pio_sim_test_warnings(test_pio_programs)
add_test(NAME test_pio_programs COMMAND test_pio_programs)

# ── libFuzzer harnesses (clang only) ─────────────────────────────────────────
# fuzz_pio_asm fuzzes the untrusted-text entry point pio_asm_assemble; fuzz_pio_sim
# fuzzes the execution core (arbitrary 16-bit words + config/stimulus via the
# public API, bypassing the assembler). The library sources are compiled *into*
# each harness with the sanitizer so ASan/UBSan instrument the internals, not just
# the harness. Seed corpora: tests/fuzz_corpus (asm, text) / tests/fuzz_corpus_sim
# (sim, binary).
option(PIO_SIM_BUILD_FUZZERS "Build the libFuzzer harnesses (requires Clang)" OFF)
if(PIO_SIM_BUILD_FUZZERS)
    if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "PIO_SIM_BUILD_FUZZERS requires Clang (libFuzzer).")
    endif()
    add_executable(fuzz_pio_asm fuzz_pio_asm.c
        ${CMAKE_SOURCE_DIR}/lib/src/pio_asm.c
        ${CMAKE_SOURCE_DIR}/lib/src/pio_sim.c)
    target_include_directories(fuzz_pio_asm PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/include ${CMAKE_SOURCE_DIR}/lib/config
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_compile_definitions(fuzz_pio_asm PRIVATE
        PIO_SIM_PIO_VERSION=${PIO_SIM_PIO_VERSION})
    target_compile_options(fuzz_pio_asm PRIVATE
        -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all -g -O1)
    target_link_options(fuzz_pio_asm PRIVATE -fsanitize=fuzzer,address,undefined)

    # fuzz_pio_sim links pio_sim.c alone — it references no other lib TU (the asm
    # harness above already links pio_asm.c + pio_sim.c without pio_gpio).
    add_executable(fuzz_pio_sim fuzz_pio_sim.c
        ${CMAKE_SOURCE_DIR}/lib/src/pio_sim.c)
    target_include_directories(fuzz_pio_sim PRIVATE
        ${CMAKE_SOURCE_DIR}/lib/include ${CMAKE_SOURCE_DIR}/lib/config
        ${CMAKE_SOURCE_DIR}/lib/src)
    target_compile_definitions(fuzz_pio_sim PRIVATE
        PIO_SIM_PIO_VERSION=${PIO_SIM_PIO_VERSION})
    target_compile_options(fuzz_pio_sim PRIVATE
        -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all -g -O1)
    target_link_options(fuzz_pio_sim PRIVATE -fsanitize=fuzzer,address,undefined)
endif()

# ── Encoding differential vs the real Raspberry Pi pioasm ────────────────────
# pio_asm is otherwise only checked against hand-derived golden encodings. When a
# pioasm binary is available, cross-check every program in tests/pio against it:
# pioasm emits one golden <base>.json at build time and test_pio_asm_vs_pioasm
# compares the 16-bit instruction words (+ wrap / side-set). Optional locally
# (pass -DPIOASM_EXECUTABLE=/path/to/pioasm); CI builds pioasm from a pinned Pico
# SDK and runs it as a gate. RP2350 (`-v 1`); base programs encode identically
# under v0.
if(NOT PIOASM_EXECUTABLE)
    find_program(PIOASM_EXECUTABLE pioasm)
endif()
if(PIOASM_EXECUTABLE)
    set(_pio_dir ${CMAKE_CURRENT_SOURCE_DIR}/pio)
    set(_gen ${CMAKE_CURRENT_BINARY_DIR}/pioasm_golden)
    file(MAKE_DIRECTORY ${_gen})
    file(GLOB _corpus RELATIVE ${_pio_dir} CONFIGURE_DEPENDS ${_pio_dir}/*.pio)
    # Files named v1_*.pio use v1-only syntax (rxfifo / jmppin / prev|next /
    # mov pindirs) that pioasm accepts only with -v 1; drop them on RP2040.
    if(PIO_SIM_PIO_VERSION LESS 1)
        list(FILTER _corpus EXCLUDE REGEX "^v1_")
    endif()
    set(_golden "")
    set(_list "")
    foreach(_pio ${_corpus})
        get_filename_component(_base ${_pio} NAME_WE)
        add_custom_command(
            OUTPUT ${_gen}/${_base}.json
            COMMAND ${PIOASM_EXECUTABLE} -o json -v ${PIO_SIM_PIO_VERSION}
                    ${_pio_dir}/${_pio} ${_gen}/${_base}.json
            DEPENDS ${_pio_dir}/${_pio}
            COMMENT "pioasm golden: ${_base}.pio (PIO v${PIO_SIM_PIO_VERSION})")
        list(APPEND _golden ${_gen}/${_base}.json)
        string(APPEND _list "    \"${_base}\",\n")
    endforeach()
    file(GENERATE OUTPUT ${_gen}/corpus_list.h
        CONTENT "/* generated by tests/CMakeLists.txt */\nstatic const char *const CORPUS[] = {\n${_list}};\n")
    add_custom_target(pioasm_golden DEPENDS ${_golden})

    add_executable(test_pio_asm_vs_pioasm test_pio_asm_vs_pioasm.c)
    add_dependencies(test_pio_asm_vs_pioasm pioasm_golden)
    target_link_libraries(test_pio_asm_vs_pioasm PRIVATE pio_sim unity)
    pio_sim_test_warnings(test_pio_asm_vs_pioasm)
    target_include_directories(test_pio_asm_vs_pioasm PRIVATE ${_gen})
    target_compile_definitions(test_pio_asm_vs_pioasm PRIVATE
        TESTS_PIO_DIR="${_pio_dir}" PIOASM_JSON_DIR="${_gen}")
    add_test(NAME test_pio_asm_vs_pioasm COMMAND test_pio_asm_vs_pioasm)
else()
    message(STATUS "pio_sim: pioasm not found; skipping the pio_asm-vs-pioasm encoding "
        "differential (pass -DPIOASM_EXECUTABLE=/path/to/pioasm to enable).")
endif()
