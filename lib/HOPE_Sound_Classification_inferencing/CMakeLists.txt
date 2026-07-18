file(GLOB_RECURSE EDGE_IMPULSE_SOURCES
    "src/*.c"
    "src/*.cpp"
    "src/*.S"
)

list(FILTER EDGE_IMPULSE_SOURCES EXCLUDE REGEX "/CMSIS/(DSP|NN)/Source/")
list(FILTER EDGE_IMPULSE_SOURCES EXCLUDE REGEX "/porting/espressif/esp-dsp/")
list(FILTER EDGE_IMPULSE_SOURCES EXCLUDE REGEX "/porting/(arduino|particle)/")
list(FILTER EDGE_IMPULSE_SOURCES EXCLUDE REGEX "/tensorflow/lite/micro/(fake_micro_context|mock_micro_graph|recording_micro_allocator|recording_single_arena_buffer_allocator|test_helper_custom_ops|test_helpers)\\.cpp$")
list(FILTER EDGE_IMPULSE_SOURCES EXCLUDE REGEX "/tensorflow/lite/micro/kernels/kernel_runner\\.cpp$")

idf_component_register(
    SRCS ${EDGE_IMPULSE_SOURCES}
    INCLUDE_DIRS
        "src"
        "src/edge-impulse-sdk/CMSIS/Core/Include"
        "src/edge-impulse-sdk/CMSIS/DSP/Include"
        "src/edge-impulse-sdk/CMSIS/NN/Include"
        "src/edge-impulse-sdk/porting/espressif/ESP-NN/include"
        "src/edge-impulse-sdk/porting/espressif/ESP-NN/src/common"
    REQUIRES esp_timer esp-dsp-fft
)

# 생성된 Edge Impulse SDK는 이전 GCC를 기준으로 작성되어 있으므로,
# GCC 15의 새 경고가 추론 코드의 빌드를 중단하지 않도록 한다.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
