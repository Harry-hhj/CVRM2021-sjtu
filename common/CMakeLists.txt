find_package(OpenCV 4 REQUIRED)

add_library(common OBJECT common.cpp)
target_link_libraries(common ${OpenCV_LIBS})
