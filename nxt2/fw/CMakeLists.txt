cmake_minimum_required(VERSION 3.8)

project(nxt_bios CXX C ASM)

set(FW_STARTUP_SOURCE_FILES
    src/startup/startup.s
    src/startup/crt.s
    src/startup/irqs.s
    src/startup/isrs.s
    src/startup/init.c
    src/startup/main.c
)

set(FW_PLATFORM_INCLUDE_FILES
    include/platform/systypes.h
    include/platform/irqs.h
    include/platform/aic.h
    include/platform/twi.h
    include/platform/systick.h
    include/platform/i2c.h
    include/platform/ports.h
    include/platform/bt.h
    include/platform/udp.h
    include/platform/uart.h
    include/platform/platform.h
)

set(FW_PLATFORM_SOURCE_FILES
    src/platform/irqs.c
    src/platform/aic.c
    src/platform/twi.c
    src/platform/systick.c
    src/platform/i2c.c
    src/platform/ports.c
    src/platform/bt.c
    src/platform/udp.c
    src/platform/uart.c
    src/platform/platform.c
)

set(FW_UTILS_HEADER_FILES
    include/utils/byte_fifo.h
    include/utils/maths.h
)

set(FW_UTILS_SOURCE_FILES
    src/utils/byte_fifo.c
)

set(FW_OS_INCLUDE_FILES
    include/os/scheduler.h
    include/os/tasks.h
    include/os/os.h
)

set(FW_OS_SOURCE_FILES
    src/os/scheduler.c
    src/os/tasks.c
    src/os/os.c
)

set(FW_DRIVERS_INCLUDE_FILES
    include/drivers/nxt_spi.h
    include/drivers/nxt_avr.h
    include/drivers/nxt_motors.h
    include/drivers/nxt_buttons.h
    include/drivers/nxt_sensors.h
    include/drivers/nxt_usb.h
    include/drivers/nxt_lcd.h
    include/drivers/nxt_display.h
    include/drivers/nxt_light_sensor.h
    include/drivers/nxt_color_sensor.h
    include/drivers/nxt_touch_sensor.h
    include/drivers/nxt_sonar_sensor.h
    include/drivers/nxt_ht_compass_sensor.h
    include/drivers/nxt_ht_gyro_sensor.h
    include/drivers/nxt_ht_ir_seeker.h
    include/drivers/nxt_ht_ir_receiver.h
    include/drivers/nxt_sound.h
    include/drivers/nxt_devices.h
)

set(FW_DRIVERS_SOURCE_FILES
    src/drivers/nxt_spi.c
    src/drivers/nxt_avr.c
    src/drivers/nxt_motors.c
    src/drivers/nxt_buttons.c
    src/drivers/nxt_sensors.c
    src/drivers/nxt_usb.c
    src/drivers/nxt_lcd.c
    src/drivers/nxt_display.c
    src/drivers/nxt_light_sensor.c
    src/drivers/nxt_color_sensor.c
    src/drivers/nxt_touch_sensor.c
    src/drivers/nxt_sonar_sensor.c
    src/drivers/nxt_ht_compass_sensor.c
    src/drivers/nxt_ht_gyro_sensor.c
    src/drivers/nxt_ht_ir_seeker.c
    #src/drivers/nxt_ht_ir_receiver.c
    src/drivers/nxt_sound.c
    src/drivers/nxt_devices.c
)

set(FW_CPP_API_INCLUDE_FILES
    include/api/nxt_touch_sensor.hpp
    include/api/nxt_distance_sensor.hpp
    include/api/nxt_light_sensor.hpp
    include/api/nxt_color_sensor.hpp
    include/api/nxt_ht_compass_sensor.hpp
    include/api/nxt_ht_gyro_sensor.hpp
    include/api/nxt_ht_ir_seeker.hpp
    include/api/nxt_buttons.hpp
    include/api/nxt_motor.hpp
    include/api/nxt_usb_data.hpp
    include/api/nxt_usb_port.hpp
    include/api/nxt_system.hpp
)

set(FW_CPP_API_SOURCE_FILES
    src/api/nxt_touch_sensor.cpp
    src/api/nxt_distance_sensor.cpp
    src/api/nxt_light_sensor.cpp
    src/api/nxt_color_sensor.cpp
    src/api/nxt_ht_compass_sensor.cpp
    src/api/nxt_ht_gyro_sensor.cpp
    src/api/nxt_ht_ir_seeker.cpp
    src/api/nxt_buttons.cpp
    src/api/nxt_motor.cpp
    src/api/nxt_usb_port.cpp
    src/api/nxt_system.cpp
)

add_library(${PROJECT_NAME} STATIC
    ${FW_STARTUP_SOURCE_FILES}
    ${FW_PLATFORM_INCLUDE_FILES}
    ${FW_PLATFORM_SOURCE_FILES}
    ${FW_OS_INCLUDE_FILES}
    ${FW_OS_SOURCE_FILES}
    ${FW_DRIVERS_INCLUDE_FILES}
    ${FW_DRIVERS_SOURCE_FILES}
    ${FW_UTILS_HEADER_FILES}
    ${FW_UTILS_SOURCE_FILES}
    ${FW_CPP_API_INCLUDE_FILES}
    ${FW_CPP_API_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        nxt_com_protocol
        nxt_common_utils
)
