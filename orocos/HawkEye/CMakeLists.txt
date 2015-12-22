#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(HawkEye)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#add_executable( ${EXE_TARGET_NAME} ${SOURCES} )

#SET( GCC_COVERAGE_COMPILE_FLAGS "pkg-config --cflags opencv" )
#SET( GCC_COVERAGE_LINK_FLAGS    "pkg-config --libs opencv")
#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
#SET( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

#set(OpenCV_DIR "/usr/lib/opencv")
#find_package(OpenCV REQUIRED core imgproc highgui)

#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(test ${OpenCV_LIBS})

#include_directories (/usr/local/include/opencv /usr/local/include)
#link_directories (/usr/local/lib)

#TARGET_LINK_LIBRARIES(-lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core -lopencv_hal)

# Rest of the configuration is in src/
add_subdirectory( src )

#set(OpenCV_DIR "/usr/local/opencv-3.0.0/build")
#set(OpenCV_DIR "/usr/local/lib")
#set(OpenCV_DIR "/home/tim/opencv-3.0.0/build")
#include_directories(${OpenCV_INCLUDE_DIRS})

#message ( STATUS "OpenCV_DIR = ${OpenCV_DIR}")
#message ( STATUS "OpenCV_DIR = ${OpenCV_DIR}")

#set(OpenCV_LIB_PATH "/usr/lib/x86_64-linux-gnu/")
#message ( STATUS "OpenCV_LIB_PATH = ${OpenCV_LIB_PATH}")

#find_package( OpenCV REQUIRED )
#if (NOT OpenCV_FOUND)
#	message (FATAL_ERROR "\nCould not find openCV")
#endif()

#message ( STATUS "OpenCV_LIBS = ${OpenCV_LIBS}" ) #no path, only last part of the path?
#message ( STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}" )

#target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
#target_link_libraries( ${EXE_TARGET_NAME} ${OpenCV_LIBS} )


message ( STATUS "OpenCV_FOUND = ${OpenCV_FOUND}" )



#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libHawkEye-types-<target>.so
# and installs in the directory lib/orocos/@target@/HawkEye/types/
#
# The header will go in include/orocos/HawkEye/types/HawkEye/HawkEye-types.hpp
# So you can #include <HawkEye/HawkEye-types.hpp>
#
#orocos_typegen_headers(include/HawkEye/HawkEye-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
