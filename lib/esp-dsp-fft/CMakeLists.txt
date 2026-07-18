idf_component_register(
    SRCS "src/dsps_fft2r.c"
    INCLUDE_DIRS "src"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
