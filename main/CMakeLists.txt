idf_component_register(
    SRCS "main.cpp"
         "autoseq.cpp"
         "audio_source.cpp"
         "ble_native.cpp"
         "core_api.cpp"
         "gps.cpp"
         "radio_control.cpp"
         "radio_control_qmx.cpp"
         "radio_control_kh1_cat.cpp"
         "ft8_audio_pipeline.cpp"
         "stream_uac.cpp"
         "stream_mic.cpp"
         "resample.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer board_cardputer_adv ft8_lib usb_host_uac usb_host_cdc_acm fatfs sdmmc spiffs bt nvs_flash efuse json
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)

# POST_BUILD DRAM budget check — fails the build if static free DIRAM falls
# below the FT8 variant threshold (50 KB). Catches silent CDC-ACM allocation
# failures caused by NimBLE consuming DRAM before we'd notice at runtime.
# Override the variant with -DDRAM_CHECK_VARIANT=ft4 for the FT4 build.
if(NOT DEFINED DRAM_CHECK_VARIANT)
    set(DRAM_CHECK_VARIANT "ft8")
endif()
add_custom_command(TARGET ${COMPONENT_LIB} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "[check_dram_budget] Running DRAM budget check (variant=${DRAM_CHECK_VARIANT})..."
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_SOURCE_DIR}/tools/check_dram_budget.py
            ${CMAKE_BINARY_DIR}
            --variant ${DRAM_CHECK_VARIANT}
    COMMENT "Checking DRAM budget"
    VERBATIM
)
