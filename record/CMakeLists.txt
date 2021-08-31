cmake_minimum_required(VERSION 3.15)

set(LIB_NAME record)

find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE src record.cpp)

add_library(${LIB_NAME} OBJECT ${src})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
