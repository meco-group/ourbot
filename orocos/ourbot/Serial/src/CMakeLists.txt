
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libSerial-<target>.so
# and installs in the directory lib/orocos/Serial/
#
orocos_component(Serial SerialInterface/SerialInterface-component.hpp SerialInterface/SerialInterface-component.cpp USBInterface/USBInterface-component.hpp USBInterface/USBInterface-component.cpp TeensyBridge/TeensyBridge-component.hpp TeensyBridge/TeensyBridge-component.cpp TeensyBridge/protocol_interface.h TeensyBridge/protocol_mavlink.h TeensyBridge/protocol_mavlink.cpp TeensyBridge/mavlink/ourbot_messages/mavlink.h TeensyBridge/IMU.hpp TeensyBridge/IMU.cpp TeensyBridge/sensor3d.hpp TeensyBridge/sensor3d.cpp RPLidar/RPLidar-component.hpp RPLidar/RPLidar-component.cpp RPLidar/rplidar_protocol.h RPLidar/rplidar_cmd.h GamePad/GamePad-component.hpp GamePad/GamePad-component.cpp GamePad/GamePad_protocol.h GamePad/GamePad_cmd.h) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Serial/ directory
#
orocos_install_headers(SerialInterface/SerialInterface-component.hpp USBInterface/USBInterface-component.hpp TeensyBridge/TeensyBridge-component.hpp TeensyBridge/protocol_interface.h TeensyBridge/protocol_mavlink.h TeensyBridge/mavlink/ourbot_messages/mavlink.h TeensyBridge/IMU.hpp TeensyBridge/sensor3d.hpp RPLidar/RPLidar-component.hpp RPLidar/rplidar_protocol.h RPLidar/rplidar_cmd.h GamePad/GamePad-component.hpp GamePad/GamePad-protocol.h GamePad/GamePad-cmd.h) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libSerial-plugin-<target>.so
# and installs in the directory lib/orocos/Serial/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Serial-plugin SerialInterface/SerialInterface-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libSerial-service-<target>.so
# and installs in the directory lib/orocos/Serial/plugins/
#
orocos_service(Serial-service SerialInterface/SerialInterface-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libSerial-support-<target>.so and installs it in
# lib/
#
orocos_library(Serial-support SerialInterface/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


