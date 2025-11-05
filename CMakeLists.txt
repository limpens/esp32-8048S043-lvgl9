cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

add_compile_definitions( LV_CONF_INCLUDE_SIMPLE=1 )
idf_build_set_property(COMPILE_OPTIONS "-I../main" APPEND)
idf_build_set_property(MINIMAL_BUILD ON)

project(lvgl9)
