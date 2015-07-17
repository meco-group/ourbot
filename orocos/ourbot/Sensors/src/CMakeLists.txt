
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libSensors-<target>.so
# and installs in the directory lib/orocos/Sensors/
#
orocos_component(Sensors Sensors-component.hpp Sensors-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Sensors/ directory
#
orocos_install_headers(Sensors-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libSensors-plugin-<target>.so
# and installs in the directory lib/orocos/Sensors/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Sensors-plugin Sensors-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libSensors-service-<target>.so
# and installs in the directory lib/orocos/Sensors/plugins/
#
orocos_service(Sensors-service Sensors-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libSensors-support-<target>.so and installs it in
# lib/
#
orocos_library(Sensors-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


