
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libSPIMaster-<target>.so
# and installs in the directory lib/orocos/SPIMaster/
#
orocos_component(SPIMaster SPIMaster/SPIMaster-component.hpp SPIMaster/SPIMaster-component.cpp SPIDeviceInterface/SPIDeviceInterface.cpp SPIDeviceInterface/SPIDeviceInterface.hpp SPIDeviceInterface/gpio.c SPIDeviceInterface/gpio.h IMU/IMU.cpp IMU/IMU.hpp IMU/sensor3d.cpp IMU/sensor3d.hpp)
#orocos_component(SPIMaster SPIDeviceInterface/SPIDeviceInterface.cpp SPIDeviceInterface/SPIDeviceInterface.hpp SPIDeviceInterface/gpio.c SPIDeviceInterface/gpio.h)
#orocos_component(SPIMaster IMU/IMU.cpp IMU/IMU.hpp IMU/sensor3d.cpp IMU/sensor3d.hpp)

# ...you may add multiple source files
#
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/SPIMaster/ directory
#
orocos_install_headers(SPIMaster/SPIMaster-component.hpp SPIDeviceInterface/SPIDeviceInterface.hpp SPIDeviceInterface/gpio.h IMU/IMU.hpp IMU/sensor3d.hpp)
#orocos_install_headers(SPIDeviceInterface/SPIDeviceInterface.hpp SPIDeviceInterface/gpio.h)
#orocos_install_headers(IMU/IMU.hpp IMU/sensor3d.hpp)
# ...you may add multiple header files
#
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libSPIMaster-plugin-<target>.so
# and installs in the directory lib/orocos/SPIMaster/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(SPIMaster-plugin SPIMaster/SPIMaster-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libSPIMaster-service-<target>.so
# and installs in the directory lib/orocos/SPIMaster/plugins/
#
orocos_service(SPIMaster-service SPIMaster/SPIMaster-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libSPIMaster-support-<target>.so and installs it in
# lib/
#
orocos_library(SPIMaster-support SPIMaster/support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


