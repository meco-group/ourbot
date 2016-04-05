#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(MotionPlanning)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Load Toolbox lib
link_directories(/home/ruben/ourbot/orocos/ourbot/MotionPlanning/src/Toolbox/bin/)
link_directories( /home/ruben/Documents/Work/Repositories/casadi_binary/casadi/ )
include_directories( /home/ruben/ourbot/orocos/ourbot/MotionPlanning/src/Toolbox/src/ )
include_directories( /home/ruben/Documents/Work/Repositories/casadi_binary/include/ )
add_definitions(-std=c++11)
# nothing should be changed in the Makefile!


# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libMotionPlanning-types-<target>.so
# and installs in the directory lib/orocos/@target@/MotionPlanning/types/
#
# The header will go in include/orocos/MotionPlanning/types/MotionPlanning/MotionPlanning-types.hpp
# So you can #include <MotionPlanning/MotionPlanning-types.hpp>
#
#orocos_typegen_headers(include/MotionPlanning/MotionPlanning-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
