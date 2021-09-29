cmake_minimum_required(VERSION 3.8)
project(example)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV 4 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 3.6 EXACT REQUIRED COMPONENTS Development Interpreter)

include_directories(../UltraMultiThread/include)

add_executable(example main.cpp)
target_link_libraries(example pthread)

target_link_libraries(example Python3::Python pybind11::pybind11)