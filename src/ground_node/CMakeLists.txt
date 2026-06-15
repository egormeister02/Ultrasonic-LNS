# ground_node — приёмный (центральный/земляной) узел: UWB-линк с air_node, далее УЗ-тайминг/якоря
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")   # общие компоненты: decadriver, dwm1000_port, uwb_protocol
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ground_node)
