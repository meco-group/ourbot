
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libEagleBridge-<target>.so
# and installs in the directory lib/orocos/EagleBridge/
#
orocos_component(EagleBridge EagleBridge-component.hpp EagleBridge-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/EagleBridge/ directory
#
orocos_install_headers(EagleBridge-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libEagleBridge-plugin-<target>.so
# and installs in the directory lib/orocos/EagleBridge/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(EagleBridge-plugin EagleBridge-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libEagleBridge-service-<target>.so
# and installs in the directory lib/orocos/EagleBridge/plugins/
#
orocos_service(EagleBridge-service EagleBridge-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libEagleBridge-support-<target>.so and installs it in
# lib/
#
orocos_library(EagleBridge-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


