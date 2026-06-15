# air_node — передающий узел БПЛА: УЗ двухимпульсный маяк + UWB-синхро/линк (ESP32-S3)
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")   # общие компоненты: decadriver, dwm1000_port, uwb_protocol
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(air_node)
