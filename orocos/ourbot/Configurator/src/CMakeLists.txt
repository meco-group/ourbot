
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libConfigurator-<target>.so
# and installs in the directory lib/orocos/Configurator/
#
orocos_component(Configurator Configurator-component.hpp Configurator-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Configurator/ directory
#
orocos_install_headers(Configurator-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libConfigurator-plugin-<target>.so
# and installs in the directory lib/orocos/Configurator/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Configurator-plugin Configurator-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libConfigurator-service-<target>.so
# and installs in the directory lib/orocos/Configurator/plugins/
#
orocos_service(Configurator-service Configurator-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libConfigurator-support-<target>.so and installs it in
# lib/
#
orocos_library(Configurator-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


