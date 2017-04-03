CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(camera_calibration)

ADD_DEFINITIONS(-std=c++11)

FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# Compile examples
add_executable(camera_calibration camera_calibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBRARIES})


