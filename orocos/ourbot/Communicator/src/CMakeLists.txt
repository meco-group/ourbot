
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libCommunicator-<target>.so
# and installs in the directory lib/orocos/Communicator/
#
orocos_component(Communicator Communicator-component.hpp Communicator-component.cpp Connection.hpp) # ...you may add multiple source files

target_link_libraries(Communicator ${ZMQ_LIBRARIES} ${CZMQ_LIBRARIES} ${ZYRE_LIBRARIES}) # Link libraries to Orocos component here

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Communicator/ directory
#
orocos_install_headers(Communicator-component.hpp Connection.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libCommunicator-plugin-<target>.so
# and installs in the directory lib/orocos/Communicator/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Communicator-plugin Communicator-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libCommunicator-service-<target>.so
# and installs in the directory lib/orocos/Communicator/plugins/
#
orocos_service(Communicator-service Communicator-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libCommunicator-support-<target>.so and installs it in
# lib/
#
orocos_library(Communicator-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


