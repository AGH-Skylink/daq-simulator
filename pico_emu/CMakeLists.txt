cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")
include(pico_sdk_import.cmake)

project(daq_pico_emu C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(daq_pico_emu main.c)
pico_generate_pio_header(daq_pico_emu ${CMAKE_CURRENT_LIST_DIR}/pdm_emu.pio)
# a host that opened the port but does not read must not stall the producer (500 ms by default)
target_compile_definitions(daq_pico_emu PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000)
target_compile_options(daq_pico_emu PRIVATE -O3 -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(daq_pico_emu pico_stdlib pico_multicore hardware_pio hardware_dma hardware_flash)

pico_enable_stdio_usb(daq_pico_emu 1)
pico_enable_stdio_uart(daq_pico_emu 0)
pico_set_program_name(daq_pico_emu "daq-sim PDM emulator")
pico_set_program_version(daq_pico_emu "0.1.0")
pico_add_extra_outputs(daq_pico_emu)
