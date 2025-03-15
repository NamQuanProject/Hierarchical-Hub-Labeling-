cmake_minimum_required(VERSION 3.10)

# Project name
project(hl)

# Set the C++ standard to C++17 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(hl hhl.cpp)