#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(Communicator)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # Find scripts for Zyre, etc.


add_definitions(-DSWM_ZYRE_CONFIG_DIR="${CMAKE_HOME_DIRECTORY}")
add_definitions(-std=c++11)

# Load zmq, czmqm zyre
find_package(ZMQ REQUIRED)
find_package(CZMQ REQUIRED)
find_package(ZYRE REQUIRED)

include_directories( ${ZMQ_INCLUDE_DIRS} ${CZMQ_INCLUDE_DIRS} ${ZYRE_INCLUDE_DIRS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )

# Return if zmq, czmqm zyre were found (=1) or not (=0)
message ( STATUS "ZMQ_FOUND = ${ZMQ_FOUND}" )
message ( STATUS "CZMQ_FOUND = ${CZMQ_FOUND}" )
message ( STATUS "ZYRE_FOUND = ${ZYRE_FOUND}" )

#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libCommunicator-types-<target>.so
# and installs in the directory lib/orocos/@target@/Communicator/types/
#
# The header will go in include/orocos/Communicator/types/Communicator/Communicator-types.hpp
# So you can #include <Communicator/Communicator-types.hpp>
#
orocos_typegen_headers(include/Communicator/Communicator-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
