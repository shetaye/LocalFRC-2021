cmake_minimum_required(VERSION 3.10)

set(ARDUINO_SDK_PATH "/Applications/Arduino.app/Contents/Java")
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/Arduino-CMake-Toolchain/Arduino-toolchain.cmake)

project(GME)
