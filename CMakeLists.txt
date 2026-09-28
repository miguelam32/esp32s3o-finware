cmake_minimum_required(VERSION 3.22)
set(IDF_TARGET esp32s3)
set(PARTITION_TABLE_CSV_PATH "${CMAKE_SOURCE_DIR}/partitions.csv")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# fix led_strip con IDF 6: fuerza esp_heap_caps.h en archivos C
idf_build_set_property(C_COMPILE_OPTIONS "-include" "esp_heap_caps.h" APPEND)
project(esp32s3o-finware)
