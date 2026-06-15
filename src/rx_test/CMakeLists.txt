# Приёмный (земляной) узел — тест датчика BME280, скорости звука и приёма UWB
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../components")   # общий компонент decadriver
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rx_test)
