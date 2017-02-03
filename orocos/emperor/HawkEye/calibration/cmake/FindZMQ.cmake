# - Try to find ZMQ
# Once done this will define
#
# ZMQ_FOUND - system has ZMQ
# ZMQ_INCLUDE_DIRS - the ZMQ include directory
# ZMQ_LIBRARIES - Link these to use ZMQ
# ZMQ_DEFINITIONS - Compiler switches required for using ZMQ
#
# Copyright (c) 2011 Lee Hambley <lee.hambley@gmail.com>
#
# Redistribution and use is allowed according to the terms of the New
# BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (ZMQ_LIBRARIES AND ZMQ_INCLUDE_DIRS)
    # in cache already
    set(ZMQ_FOUND TRUE)
else (ZMQ_LIBRARIES AND ZMQ_INCLUDE_DIRS)

find_path(ZMQ_INCLUDE_DIR
    NAMES
    zmq.h
    HINTS
    /opt/zeromq-4.1.2/include
    PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    /sw/include    
    ${ZMQ_ROOT}/include
    ENV{ZMQ_ROOT}/include
)

find_library(ZMQ_LIBRARY
    NAMES
    zmq libzmq
    HINTS
    /opt/zeromq-4.1.2/lib
    PATH
    /usr/lib
    /usr/local/lib
    /opt/local/lib
    /sw/lib
    ${ZMQ_ROOT}/lib
    ENV{ZMQ_ROOT}/lib
)

set(ZMQ_INCLUDE_DIRS
    ${ZMQ_INCLUDE_DIR}
)

if (ZMQ_LIBRARY)
    set(ZMQ_LIBRARIES
        ${ZMQ_LIBRARIES}
        ${ZMQ_LIBRARY}
)
endif (ZMQ_LIBRARY)

# show the ZMQ_INCLUDE_DIRS and ZMQ_LIBRARIES variables only in the advanced view
mark_as_advanced(FORCE ZMQ_INCLUDE_DIRS ZMQ_LIBRARIES)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZMQ DEFAULT_MSG ZMQ_LIBRARIES ZMQ_INCLUDE_DIRS)

endif (ZMQ_LIBRARIES AND ZMQ_INCLUDE_DIRS)

