
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libControllerInterface-<target>.so
# and installs in the directory lib/orocos/ControllerInterface/
#
orocos_component(ControllerInterface ControllerInterface/ControllerInterface-component.hpp ControllerInterface/ControllerInterface-component.cpp DistributedControllerInterface/DistributedControllerInterface-component.cpp DistributedControllerInterface/DistributedControllerInterface-component.hpp ExampleController/ExampleController-component.hpp ExampleController/ExampleController-component.cpp)
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/ControllerInterface/ directory
#
orocos_install_headers(ControllerInterface/ControllerInterface-component.hpp DistributedControllerInterface/DistributedControllerInterface-component.hpp ExampleController/ExampleController-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libControllerInterface-plugin-<target>.so
# and installs in the directory lib/orocos/ControllerInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(ControllerInterface-plugin ControllerInterface/ControllerInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libControllerInterface-service-<target>.so
# and installs in the directory lib/orocos/ControllerInterface/plugins/
#
orocos_service(ControllerInterface-service ControllerInterface/ControllerInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libControllerInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(ControllerInterface-support ControllerInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


