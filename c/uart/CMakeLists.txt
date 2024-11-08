cmake_minimum_required(VERSION 3.13)
if (DEFINED ENV{PICO_SDK_PATH})
  include("$ENV{PICO_SDK_PATH}/pico_sdk_init.cmake")
else ()
  include("../pico-sdk/pico_sdk_init.cmake")
endif()
project(project C CXX ASM)
add_compile_options(-Wall -Wextra)
pico_sdk_init()
add_subdirectory(../pololu_3pi_2040_robot pololu_3pi_2040_robot)




add_executable(uart uart.c)
pico_enable_stdio_usb(uart 1)
pico_enable_stdio_uart(uart 0)
pico_add_extra_outputs(uart)
target_link_libraries(uart pico_stdlib)
target_link_libraries(uart pico_time)
target_link_libraries(uart pololu_3pi_2040_robot)

pico_generate_pio_header(uart ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
