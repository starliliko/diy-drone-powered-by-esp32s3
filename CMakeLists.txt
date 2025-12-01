# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(__PROJECT_GROUP_LINK_COMPONENTS ON)

set(EXTRA_COMPONENT_DIRS
    components/core
    components/drivers
    components/drivers/general
    components/drivers/i2c_devices
    components/drivers/spi_devices
    components/lib
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESPDrone)