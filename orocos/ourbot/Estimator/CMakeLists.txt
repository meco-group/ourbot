#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(Estimator)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_library(SLICOT_LIB
  slicot
  PATHS $ENV{SLICOT})

if(NOT SLICOT_LIB)
  message(STATUS "SLICOT: Cound not find library slicot. Try stetting SLICOT_LIBRARY_DIR env var.")
endif()

add_definitions(-std=c++11)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libEstimator-types-<target>.so
# and installs in the directory lib/orocos/@target@/Estimator/types/
#
# The header will go in include/orocos/Estimator/types/Estimator/Estimator-types.hpp
# So you can #include <Estimator/Estimator-types.hpp>
#
#orocos_typegen_headers(include/Estimator/Estimator-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
