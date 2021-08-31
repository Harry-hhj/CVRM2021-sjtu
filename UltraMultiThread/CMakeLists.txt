cmake_minimum_required(VERSION 3.12)
project(umt)

set(CMAKE_CXX_STANDARD 17)

find_package(pybind11 REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)

include_directories(include)

option(WITH_EXAMPLE OFF)
if(WITH_EXAMPLE)
    add_executable(example example/example.cpp)
    target_link_libraries(example Python::Python pybind11::pybind11)
endif()