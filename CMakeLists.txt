# f16 dynamics library
cmake_minimum_required(VERSION 3.16)
project(f16_flight_dynamics)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall")

find_package(Boost 1.71 COMPONENTS numpy REQUIRED)

# main library
add_subdirectory(src)

# python library
add_subdirectory(python-bindings)
