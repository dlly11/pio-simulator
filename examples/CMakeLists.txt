# SPDX-License-Identifier: MIT
# Reference example programs. Each file is a standalone main() linked against
# pio_sim; enable with -DPIO_SIM_BUILD_EXAMPLES=ON. They are built and run in CI
# so they never drift from the public API.
set(PIO_SIM_EXAMPLES
    01_blink
    02_fifo_echo
    03_uart_tx
    04_ws2812
    05_spi_dma
    06_chip_bus)

foreach(example ${PIO_SIM_EXAMPLES})
    add_executable(${example} ${example}.c)
    target_link_libraries(${example} PRIVATE pio_sim)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
        # Match the library: warnings are errors only for a standalone build, so
        # embedders on a different compiler aren't broken by its warning set.
        if(PROJECT_IS_TOP_LEVEL)
            target_compile_options(${example} PRIVATE -Werror)
        endif()
    endif()
endforeach()
