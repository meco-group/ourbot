
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libTestCorba-<target>.so
# and installs in the directory lib/orocos/TestCorba/
#
orocos_component(TestCorba TestCorba-component.hpp TestCorba-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/TestCorba/ directory
#
orocos_install_headers(TestCorba-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libTestCorba-plugin-<target>.so
# and installs in the directory lib/orocos/TestCorba/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(TestCorba-plugin TestCorba-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libTestCorba-service-<target>.so
# and installs in the directory lib/orocos/TestCorba/plugins/
#
orocos_service(TestCorba-service TestCorba-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libTestCorba-support-<target>.so and installs it in
# lib/
#
orocos_library(TestCorba-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


