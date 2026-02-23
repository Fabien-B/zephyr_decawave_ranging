# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_dwm3001c)

target_sources(app PRIVATE
    src/main.c
    src/pdec_common.c
    src/pdec_initiator.c
    src/pdec_database.c
)
