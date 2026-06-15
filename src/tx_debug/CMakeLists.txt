# tx_debug — отладочный передатчик: двухимпульсный УЗ-маяк без UWB-протокола (для настройки аналогового тракта)
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")   # decadriver, dwm1000_port
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tx_debug)
