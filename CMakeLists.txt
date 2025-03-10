cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(helloWorld C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(lecture02
    Lecture02-24-25.c
)

pico_enable_stdio_usb(lecture02 1)
pico_enable_stdio_uart(lecture02 0)
pico_add_extra_outputs(lecture02)
target_link_libraries(lecture02 pico_stdlib hardware_i2c hardware_pwm)