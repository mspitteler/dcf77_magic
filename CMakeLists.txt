cmake_minimum_required(VERSION 3.13)

add_executable(dcf77_magic
        main.c
        )
        
# initialize the SDK directly
include(/home/mattijn/.local/raspberrypi/pico-sdk/pico_sdk_init.cmake)

project(dcf77_magic)

pico_sdk_init()

target_link_libraries(dcf77_magic pico_stdlib
        hardware_adc
        hardware_dma
        hardware_pwm
        # For the dummy output:
        hardware_pio
        hardware_rtc
        pico_multicore)
target_compile_definitions(dcf77_magic PRIVATE
        PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=2000)

set_property(TARGET dcf77_magic PROPERTY C_STANDARD 23)

pico_enable_stdio_usb(dcf77_magic 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(dcf77_magic)

# add url via pico_set_program_url
# example_auto_set_url(dcf77_magic)

