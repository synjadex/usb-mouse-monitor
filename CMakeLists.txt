cmake_minimum_required(VERSION 4.0)
project(usb_mouse_monitor C)

set(CMAKE_C_STANDARD 11)

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_executable(usb_mouse_monitor main.c)

target_link_libraries(usb_mouse_monitor PRIVATE setupapi kernel32 user32)
