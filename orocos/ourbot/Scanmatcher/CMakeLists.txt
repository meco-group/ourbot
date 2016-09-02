#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(Scanmatcher)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(PkgConfig REQUIRED)
# Tell pkgconfig to look for CSM
pkg_check_modules(CSM REQUIRED csm)

IF(${CSM_FOUND})
        MESSAGE("CSM_LIBRARY_DIRS: ${CSM_LIBRARY_DIRS}")
        MESSAGE("CSM_LIBRARIES: ${CSM_LIBRARIES}")
        MESSAGE("CSM_INCLUDE_DIRS: ${CSM_INCLUDE_DIRS}")

        INCLUDE_DIRECTORIES(${CSM_INCLUDE_DIRS}) # important!
        LINK_DIRECTORIES(${CSM_LIBRARY_DIRS})    # important!
ELSE(${CSM_FOUND})
        MESSAGE(FATAL_ERROR "CSM not found. Check that the environment variable PKG_CONFIG_PATH includes the path containing the file 'csm.pc'.")
ENDIF(${CSM_FOUND})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
add_subdirectory( src )
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libScanmatcher-types-<target>.so
# and installs in the directory lib/orocos/@target@/Scanmatcher/types/
#
# The header will go in include/orocos/Scanmatcher/types/Scanmatcher/Scanmatcher-types.hpp
# So you can #include <Scanmatcher/Scanmatcher-types.hpp>
#
#orocos_typegen_headers(include/Scanmatcher/Scanmatcher-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
