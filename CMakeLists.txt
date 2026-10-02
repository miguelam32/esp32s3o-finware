cmake_minimum_required(VERSION 3.22)

set(IDF_TARGET esp32s3)

# La tabla de particiones y el rendimiento se configuran en sdkconfig.defaults
# (CONFIG_PARTITION_TABLE_CUSTOM, etc.), no con variables de CMake.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3o-finware)

