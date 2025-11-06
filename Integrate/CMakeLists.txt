# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)
set(EXTRA_COMPONENT_DIRS
    # Add local esp-zigbee-sdk components and example common dirs so Zigbee headers/components
    # are visible to this project. Paths are relative to Integrate/CMakeLists.txt.
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp-zigbee-sdk/components
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp-zigbee-sdk/examples/common/switch_driver
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp-zigbee-sdk/examples/common/zcl_utility
    )
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
project(Integrate)