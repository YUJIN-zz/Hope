idf_component_register(
    SRCS "main.cpp" "direction_estimate.c"
    INCLUDE_DIRS "."
    REQUIRES esp-dsp driver
)
