
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libVelocityCommandInterface-<target>.so
# and installs in the directory lib/orocos/VelocityCommandInterface/
#
orocos_component(VelocityCommandInterface VelocityCommandInterface/VelocityCommandInterface-component.hpp VelocityCommandInterface/VelocityCommandInterface-component.cpp)
orocos_component(VelocityCommandInterface ExampleVelocityCommand/ExampleVelocityCommand-component.hpp ExampleVelocityCommand/ExampleVelocityCommand-component.cpp)
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/VelocityCommandInterface/ directory
#
orocos_install_headers(VelocityCommandInterface/VelocityCommandInterface-component.hpp)
orocos_install_headers(ExampleVelocityCommand/ExampleVelocityCommand-component.hpp)
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libVelocityCommandInterface-plugin-<target>.so
# and installs in the directory lib/orocos/VelocityCommandInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(VelocityCommandInterface-plugin VelocityCommandInterface/VelocityCommandInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libVelocityCommandInterface-service-<target>.so
# and installs in the directory lib/orocos/VelocityCommandInterface/plugins/
#
#orocos_service(VelocityCommandInterface-service VelocityCommandInterface/VelocityCommandInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libVelocityCommandInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(VelocityCommandInterface-support VelocityCommandInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


