
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libDummy-<target>.so
# and installs in the directory lib/orocos/Dummy/
#
orocos_component(Dummy Dummy-component.hpp Dummy-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Dummy/ directory
#
orocos_install_headers(Dummy-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libDummy-plugin-<target>.so
# and installs in the directory lib/orocos/Dummy/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Dummy-plugin Dummy-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libDummy-service-<target>.so
# and installs in the directory lib/orocos/Dummy/plugins/
#
orocos_service(Dummy-service Dummy-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libDummy-support-<target>.so and installs it in
# lib/
#
orocos_library(Dummy-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


