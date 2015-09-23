
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libVelocityCommandEmperorInterface-<target>.so
# and installs in the directory lib/orocos/VelocityCommandEmperorInterface/
#
orocos_component(VelocityCommandEmperorInterface VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-component.hpp VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-component.cpp GamePadCommand/GamePadCommand-component.hpp GamePadCommand/GamePadCommand-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/VelocityCommandEmperorInterface/ directory
#
orocos_install_headers(VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-component.hpp GamePadCommand/GamePadCommand-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libVelocityCommandEmperorInterface-plugin-<target>.so
# and installs in the directory lib/orocos/VelocityCommandEmperorInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(VelocityCommandEmperorInterface-plugin VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libVelocityCommandEmperorInterface-service-<target>.so
# and installs in the directory lib/orocos/VelocityCommandEmperorInterface/plugins/
#
orocos_service(VelocityCommandEmperorInterface-service VelocityCommandEmperorInterface/VelocityCommandEmperorInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libVelocityCommandEmperorInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(VelocityCommandEmperorInterface-support VelocityCommandEmperorInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


