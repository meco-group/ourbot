
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libController-<target>.so
# and installs in the directory lib/orocos/Controller/
#
orocos_component(Controller ControllerInterface/ControllerInterface-component.hpp ControllerInterface/ControllerInterface-component.cpp FfwController/FfwController-component.hpp FfwController/FfwController-component.cpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Controller/ directory
#
orocos_install_headers(ControllerInterface/ControllerInterface-component.hpp FfwController/FfwController-component.hpp)
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libController-plugin-<target>.so
# and installs in the directory lib/orocos/Controller/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Controller-plugin ControllerInterface/ControllerInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libController-service-<target>.so
# and installs in the directory lib/orocos/Controller/plugins/
#
orocos_service(Controller-service ControllerInterface/ControllerInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libController-support-<target>.so and installs it in
# lib/
#
orocos_library(Controller-support ControllerInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


