
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libSerialInterface-<target>.so
# and installs in the directory lib/orocos/SerialInterface/
#
orocos_component(SerialInterface SerialInterface/SerialInterface-component.hpp SerialInterface/SerialInterface-component.cpp USBInterface/USBInterface-component.hpp USBInterface/USBInterface-component.cpp  GamePad/GamePad-component.hpp GamePad/GamePad-component.cpp GamePad/GamePad_protocol.h GamePad/GamePad_cmd.h) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/SerialInterface/ directory
#
orocos_install_headers(SerialInterface/SerialInterface-component.hpp USBInterface/USBInterface-component.hpp GamePad/GamePad-component.hpp GamePad/GamePad-protocol.h GamePad/GamePad-cmd.h) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libSerialInterface-plugin-<target>.so
# and installs in the directory lib/orocos/SerialInterface/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(SerialInterface-plugin SerialInterface/SerialInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libSerialInterface-service-<target>.so
# and installs in the directory lib/orocos/SerialInterface/plugins/
#
orocos_service(SerialInterface-service SerialInterface/SerialInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libSerialInterface-support-<target>.so and installs it in
# lib/
#
orocos_library(SerialInterface-support SerialInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


