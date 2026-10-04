cmake_minimum_required(VERSION 4.0)
project(usb_mouse_monitor C)

set(CMAKE_C_STANDARD 11)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_executable(usb_mouse_monitor src/main.c)
