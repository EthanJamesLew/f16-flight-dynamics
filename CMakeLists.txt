# f16 dynamics library
cmake_minimum_required(VERSION 3.16)

# main project\
project(
        f16_flight_dynamics
        VERSION "0.1.0"
        LANGUAGES CXX
        )

# C++ config
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# boost requirement
find_package(Boost COMPONENTS numpy REQUIRED)

# main library
add_subdirectory(src)

# python library
add_subdirectory(python-bindings)
