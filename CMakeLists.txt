cmake_minimum_required(VERSION 3.13)
set(project_name dma_for_teensy)
project(${project_name})

set(CMAKE_CXX_STANDARD 14)

add_executable(${project_name}
        DmaTeensy3_6.ino
        kinetis.h
        DMAChannel.cpp DMAChannel.h
        HardwareSerial.h HardwareSerial1.cpp HardwareSerial2.cpp HardwareSerial3.cpp HardwareSerial4.cpp
        HardwareSerial5.cpp HardwareSerial6.cpp
        core_pins.h
        serial1.c serial2.c serial3.c serial4.c serial5.c serial6.c serial6_lpuart.c
        DmaSerialTeensy.cpp DmaSerialTeensy.h
        Stream.cpp Stream.h
        Print.cpp Print.h
        )

add_definitions(-D__MK66FX1M0__ -DTEENSYDUINO=148 -DF_CPU=180000000 -D__cplusplus)
