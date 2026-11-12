# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_dwm3001c)

# Replace only the platform hardware port: the upstream port dispatches dwt_isr()
# on the system workqueue. The application's radio thread must own those calls.
set(dw3000_module_dir "${ZEPHYR_ZEPHYR_DW3000_DECADRIVER_MODULE_DIR}")
set_source_files_properties(
    "${dw3000_module_dir}/platform/dw3000_hw.c"
    DIRECTORY "${dw3000_module_dir}"
    PROPERTIES HEADER_FILE_ONLY TRUE
)

target_sources(app PRIVATE
    src/app_print.c
    src/main.c
    src/pdec_common.c
    src/pdec_radio.c
    src/pdec_dw3000_hw.c
    src/pdec_discovery.c
    src/pdec_initiator.c
    src/pdec_database.c
    src/pdec_ranging.c
    src/pdec_range_request.c
    src/pdec_tdoa.c
    src/uart_binary.c
    src/uart_protocol.c
    src/uart_shell.c

    src/ble.c
)
