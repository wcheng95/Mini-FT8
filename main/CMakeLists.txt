idf_component_register(
    SRCS "main.cpp"
         "tx_queue.cpp"
         "stream_wav.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer ft8_lib bt
    PRIV_REQUIRES spiffs
)
