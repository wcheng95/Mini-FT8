idf_component_register(SRCS
    ft8/decode.c
    ft8/message.c
    ft8/text.c
    ft8/encode.c
    ft8/constants.c
    ft8/crc.c
    ft8/ldpc.c
    fft/kiss_fft.c
    fft/kiss_fftr.c
    common/monitor.c
    common/fft_wrapper.c
    # add any other ft8_lib sources you need
    INCLUDE_DIRS "."
)

# Relax warnings that trigger -Werror in IDF
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-const-variable
    -Wno-format
)
