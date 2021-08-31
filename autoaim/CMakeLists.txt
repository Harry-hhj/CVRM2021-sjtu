cmake_minimum_required(VERSION 3.15)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(TensorRT REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres 2 REQUIRED)
find_package(fmt REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TensorRT_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

aux_source_directory(detector detector_src)
aux_source_directory(predictor predictor_src)
aux_source_directory(antitop antitop_src)

add_library(autoaim OBJECT autoaim.cpp ${detector_src} ${predictor_src} ${antitop_src} ${TensorRT_SOURCE})
target_link_libraries(autoaim ${OpenCV_LIBS} ${TensorRT_LIBS} ${CERES_LIBRARIES} fmt::fmt nvonnxparser cuda)
