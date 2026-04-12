file(GLOB SRCS
     src/*.cpp
     src/utility/*.cpp
     src/utility/imu/*.cpp
     src/utility/led/*.cpp
     src/utility/power/*.cpp
     src/utility/rtc/*.cpp
     )

if (IDF_VERSION_MAJOR GREATER_EQUAL 5)
    set(COMPONENT_REQUIRES M5GFX esp_adc driver)
else()
    set(COMPONENT_REQUIRES M5GFX esp_adc_cal)
endif()

### If you use arduino-esp32 components, please activate next comment line.
# list(APPEND COMPONENT_REQUIRES arduino-esp32)

message(STATUS "M5Unified use component = ${COMPONENT_REQUIRES}" )

if(COMMAND idf_component_register)
    idf_component_register(SRCS ${SRCS} INCLUDE_DIRS src REQUIRES ${COMPONENT_REQUIRES})
else()
    set(COMPONENT_ADD_INCLUDEDIRS src)
    set(COMPONENT_SRCS ${SRCS})
    register_component()
endif()
