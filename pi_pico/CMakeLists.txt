# PROYECTO: SEGUIDOR DE LINEA PID 5 SENSORES
# FECHA: 11/07/2025
# MGU2508

cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(lfpro
LANGUAGES CXX C ASM
VERSION 1.0.0.0
)

pico_sdk_init()

include_directories(
    PicoPWM/include
)

add_subdirectory(PicoPWM)

add_executable(${PROJECT_NAME} lfpro.cpp)
target_link_libraries(${PROJECT_NAME} pico_stdlib PicoPWM)

pico_add_extra_outputs(${PROJECT_NAME})

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
