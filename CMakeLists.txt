# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mini_ft8)
spiffs_create_partition_image(spiffs ${CMAKE_SOURCE_DIR}/spiffs_image FLASH_IN_PROJECT)
