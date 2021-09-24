cmake_minimum_required(VERSION 3.10)
project(CVRM2021)

set(CMAKE_CXX_STANDARD 20)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif ()

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")

add_compile_options(-Wall -Werror=return-type -Wno-unused-variable)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    add_compile_options(-O3 -flto -march=native)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    add_compile_options(-O3 -flto -march=native -g)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    add_compile_options(-Os -flto -march=native)
else ()
    add_compile_options(-O0 -g)
endif ()


find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)

include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
include_directories(UltraMultiThread/include)

add_executable(CVRM2021 main.cpp)
target_link_libraries(CVRM2021 Python::Python pybind11::pybind11)

include_directories(common)
add_subdirectory(common)
target_link_libraries(CVRM2021 common)

include_directories(core_io)
add_subdirectory(core_io)
target_link_libraries(CVRM2021 core_io)

include_directories(autoaim)
add_subdirectory(autoaim)
target_link_libraries(CVRM2021 autoaim)

include_directories(record)
add_subdirectory(record)
target_link_libraries(CVRM2021 record)
