
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libcontainer-<target>.so
# and installs in the directory lib/orocos/container/
#
orocos_component(container container-component.hpp container-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/container/ directory
#
orocos_install_headers(container-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libcontainer-plugin-<target>.so
# and installs in the directory lib/orocos/container/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(container-plugin container-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libcontainer-service-<target>.so
# and installs in the directory lib/orocos/container/plugins/
#
orocos_service(container-service container-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libcontainer-support-<target>.so and installs it in
# lib/
#
orocos_library(container-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


