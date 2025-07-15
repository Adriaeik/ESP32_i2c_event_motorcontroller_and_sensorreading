# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
# Skru av -Werror for å unngå at warning stoppar bygging
add_compile_options(-Wno-error)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(i2c_motorcontroller)
