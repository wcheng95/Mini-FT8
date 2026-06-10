# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mini_ft8)

# POST_BUILD DRAM budget check — fails the build if static free DIRAM falls
# below the FT8 variant threshold (50 KB). Catches silent CDC-ACM allocation
# failures caused by NimBLE consuming DRAM before we'd notice at runtime.
# Override the variant with -DDRAM_CHECK_VARIANT=ft4 for the FT4 build.
# Hooked to the `app` target (post-link) so idf-size sees the final ELF.
if(NOT DEFINED DRAM_CHECK_VARIANT)
    set(DRAM_CHECK_VARIANT "ft8")
endif()
add_custom_command(TARGET app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "[check_dram_budget] Running DRAM budget check (variant=${DRAM_CHECK_VARIANT})..."
    COMMAND ${PYTHON}
            ${CMAKE_SOURCE_DIR}/tools/check_dram_budget.py
            ${CMAKE_BINARY_DIR}
            --variant ${DRAM_CHECK_VARIANT}
    COMMENT "Checking DRAM budget"
    VERBATIM
)

# Normal idf.py flash preserves the existing storage partition. The merged
# release image also leaves SPIFFS untouched; first boot after repartition can
# format SPIFFS if mount fails.
add_custom_command(TARGET app POST_BUILD
    COMMAND ${PYTHON} -m esptool --chip esp32s3 merge_bin -o "MiniFT8_Merged_Auto.bin" @flash_args
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Auto-generating merged binary MiniFT8_Merged_Auto.bin for release..."
)
