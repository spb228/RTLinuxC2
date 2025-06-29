cmake_minimum_required(VERSION 3.10)

project(RTLinuxC2)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)
set(CMAKE_C_EXTENSIONS OFF)

add_subdirectory(../src src)

MESSAGE ("Cmake project build successfully")