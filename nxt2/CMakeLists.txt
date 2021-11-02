cmake_minimum_required(VERSION 3.5)

project(nxt_bios CXX C ASM)

include(arm-gcc-toolchain)

set(APP_NAME "appname" CACHE STRING "Application to build")
set(LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/nxt.lds)

string(CONCAT C_FLAGS
    " -Wall"
    " -Werror"
    " -Wno-unused-variable"
    " -Wno-unused-but-set-variable" # HW register read
    " -Werror=implicit-function-declaration")

string(CONCAT CXX_FLAGS
    " -Wall"
    " -Werror"
    " -Wno-write-strings")

string(CONCAT LINKER_FLAGS
    " -T${LINKER_FILE}"
    " -Wl,-Map,${PROJECT_NAME}.map")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LINKER_FLAGS}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(platform drivers rtos apps)

# Get application source files
#include(apps/common.cmake)
#include(apps/${APP_NAME}/${APP_NAME}.cmake)

set(FW_STARTUP_SOURCE_FILES
    source/startup/startup.s
    source/startup/crt.s
    source/startup/irqs.s
    source/startup/isrs.s
    source/startup/init.c
    source/startup/main.c
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
    source/platform/irqs.c
    source/platform/aic.c
    source/platform/twi.c
    source/platform/systick.c
    source/platform/i2c.c
    source/platform/ports.c
    source/platform/bt.c
    source/platform/udp.c
    source/platform/uart.c
    source/platform/platform.c
)

set(FW_UTILS_HEADER_FILES
    include/utils/byte_fifo.h
    include/utils/maths.h
)

set(FW_UTILS_SOURCE_FILES
    source/utils/byte_fifo.c
)

set(FW_OS_INCLUDE_FILES
    include/os/scheduler.h
    include/os/tasks.h
    include/os/os.h
)

set(FW_OS_SOURCE_FILES
    source/os/scheduler.c
    source/os/tasks.c
    source/os/os.c
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
    include/drivers/nxt_ht_ir_seeker.h
    include/drivers/nxt_ht_ir_receiver.h
    include/drivers/nxt_devices.h
)

set(FW_DRIVERS_SOURCE_FILES
    source/drivers/nxt_spi.c
    source/drivers/nxt_avr.c
    source/drivers/nxt_motors.c
    source/drivers/nxt_buttons.c
    source/drivers/nxt_sensors.c
    source/drivers/nxt_usb.c
    source/drivers/nxt_lcd.c
    source/drivers/nxt_display.c
    source/drivers/nxt_light_sensor.c
    source/drivers/nxt_color_sensor.c
    source/drivers/nxt_touch_sensor.c
    source/drivers/nxt_sonar_sensor.c
    source/drivers/nxt_ht_compass_sensor.c
    source/drivers/nxt_ht_ir_seeker.c
    #source/drivers/nxt_ht_ir_receiver.c
    source/drivers/nxt_devices.c
)

set(FW_CPP_API_INCLUDE_FILES
    include/api/nxt_port_enum.hpp
    include/api/nxt_touch_sensor.hpp
    include/api/nxt_distance_sensor.hpp
    include/api/nxt_light_sensor.hpp
    include/api/nxt_color_sensor.hpp
    include/api/nxt_ht_compass_sensor.hpp
    include/api/nxt_ht_ir_seeker.hpp
    include/api/nxt_buttons.hpp
    include/api/nxt_sensing_motor.hpp
    include/api/nxt_usb_data.hpp
    include/api/nxt_usb_port.hpp
)

set(FW_CPP_API_SOURCE_FILES
    source/api/nxt_touch_sensor.cpp
    source/api/nxt_distance_sensor.cpp
    source/api/nxt_light_sensor.cpp
    source/api/nxt_color_sensor.cpp
    source/api/nxt_ht_compass_sensor.cpp
    source/api/nxt_ht_ir_seeker.cpp
    source/api/nxt_buttons.cpp
    source/api/nxt_sensing_motor.cpp
    source/api/nxt_usb_port.cpp
)

add_executable(${PROJECT_NAME}
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
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE NXT_DUMMY_APP_BG_TASK NXT_DUMMY_APP_INIT
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
generate_object(${PROJECT_NAME})
