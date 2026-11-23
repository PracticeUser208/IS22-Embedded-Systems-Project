cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)

project(imu_bringup C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(imu_bringup
    main_bringup.c
    imu_port_pico_bare.c
    ../imu.c
    ../imu_logic.c
)
target_compile_definitions(imu_bringup PRIVATE IMU_ENABLE_DEBUG=1)
target_include_directories(imu_bringup PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(imu_bringup pico_stdlib hardware_i2c hardware_gpio m)

pico_enable_stdio_usb(imu_bringup 1)    # serial over the USB cable
pico_enable_stdio_uart(imu_bringup 0)
pico_add_extra_outputs(imu_bringup)     # makes the .uf2
