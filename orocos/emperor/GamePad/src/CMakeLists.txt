
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libGamePad-<target>.so
# and installs in the directory lib/orocos/GamePad/
#
orocos_component(GamePad GamePad-component.hpp GamePad-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/GamePad/ directory
#
orocos_install_headers(GamePad-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libGamePad-plugin-<target>.so
# and installs in the directory lib/orocos/GamePad/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(GamePad-plugin GamePad-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libGamePad-service-<target>.so
# and installs in the directory lib/orocos/GamePad/plugins/
#
orocos_service(GamePad-service GamePad-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libGamePad-support-<target>.so and installs it in
# lib/
#
orocos_library(GamePad-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


