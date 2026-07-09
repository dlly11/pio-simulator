# SPDX-License-Identifier: MIT
# Minimal downstream project that consumes an *installed* pio_sim via
# find_package. CI builds this against a fresh `cmake --install` prefix to
# prove the packaging works end to end.
cmake_minimum_required(VERSION 3.21)
project(pio_sim_consumer LANGUAGES C)

find_package(pio_sim CONFIG REQUIRED)

add_executable(consumer main.c)
target_link_libraries(consumer PRIVATE pio_sim::pio_sim)
