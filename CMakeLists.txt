# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
# Include FreeRTOS Kernel
set(FREERTOS_KERNEL_PATH "X:/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(sensores_dual_core C CXX ASM)
pico_sdk_init()


include_directories( ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/lib
    ${PICO_SDK_PATH}
    )

add_executable(${PROJECT_NAME}  
        sensores_dual_core.c 
        lib/aht20.c 
        lib/bmp280.c 
        lib/ssd1306.c
        )

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib 
        pico_multicore
        hardware_i2c
        hardware_gpio
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
        pico_mbedtls
        pico_lwip_mbedtls
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(${PROJECT_NAME})
