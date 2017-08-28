
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libScanmatcher-<target>.so
# and installs in the directory lib/orocos/Scanmatcher/
#
orocos_component(Scanmatcher Scanmatcher-component.hpp Scanmatcher-component.cpp ./Environment/Circle.hpp ./Environment/Circle.cpp ./Environment/Polygon.hpp ./Environment/Polygon.cpp) # ...you may add multiple source files
#
target_link_libraries(Scanmatcher ${CSM_LIBRARIES})

# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/Scanmatcher/ directory
#
orocos_install_headers(Scanmatcher-component.hpp rapidxml.hpp ./Environment/Circle.hpp ./Environment/Polygon.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libScanmatcher-plugin-<target>.so
# and installs in the directory lib/orocos/Scanmatcher/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
orocos_plugin(Scanmatcher-plugin Scanmatcher-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libScanmatcher-service-<target>.so
# and installs in the directory lib/orocos/Scanmatcher/plugins/
#
orocos_service(Scanmatcher-service Scanmatcher-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libScanmatcher-support-<target>.so and installs it in
# lib/
#
orocos_library(Scanmatcher-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


