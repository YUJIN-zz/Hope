cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/lib/esp-dsp-fft"
    "${CMAKE_CURRENT_LIST_DIR}/lib/HOPE_Sound_Classification_inferencing"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hopeproject)
