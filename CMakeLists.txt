cmake_minimum_required(VERSION 3.13)

add_executable(adc_console
        adc_console.c
        )
        
# initialize the SDK directly
include(/home/mattijn/.local/raspberrypi/pico-sdk/pico_sdk_init.cmake)

project(adc_console)

pico_sdk_init()

target_link_libraries(adc_console pico_stdlib
        hardware_adc
        hardware_dma
        # For the dummy output:
        hardware_pio
        pico_multicore)

pico_enable_stdio_usb(adc_console 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(adc_console)

# add url via pico_set_program_url
# example_auto_set_url(adc_console)

