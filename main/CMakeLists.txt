idf_component_register(
    SRCS "main.cpp"
         "autoseq.cpp"
         "audio_source.cpp"
         "radio_control.cpp"
         "radio_control_qmx.cpp"
         "radio_control_kh1_cat.cpp"
         "stream_uac.cpp"
         "resample.cpp"
         "gps.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer ft8_lib usb_host_uac usb_host_cdc_acm fatfs sdmmc bt nvs_flash efuse
    PRIV_REQUIRES spiffs
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror)
