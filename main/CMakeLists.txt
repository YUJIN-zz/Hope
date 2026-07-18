idf_component_register(
    SRCS "main.cpp" "direction_estimate.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp-dsp-fft HOPE_Sound_Classification_inferencing
)

# Edge Impulse SDK와 ESP-IDF 6의 GCC 15 옵션 조합에서 발생하는
# 언어별 옵션 경고를 실제 컴파일 오류로 승격하지 않는다.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
