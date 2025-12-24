# CMakeLists.txt for M5Cardputer ESP-IDF component
# This is an Arduino library adapted for ESP-IDF

idf_component_register(
    SRCS
        "src/M5Cardputer.cpp"
        "src/utility/Adafruit_TCA8418/Adafruit_TCA8418.cpp"
        "src/utility/Keyboard/Keyboard.cpp"
        "src/utility/Keyboard/KeyboardReader/TCA8418.cpp"
        "src/utility/Keyboard/KeyboardReader/IOMatrix.cpp"
    INCLUDE_DIRS
        "src"
        "src/utility"
        "src/utility/Adafruit_TCA8418"
        "src/utility/Keyboard"
        "src/utility/Keyboard/KeyboardReader"
    REQUIRES
        M5Unified
        M5GFX
)
