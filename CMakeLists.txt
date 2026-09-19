# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# FT4 support — compile-time opt-in (default: ON).
#
# When ON, FT4 protocol code is included and the user can select FT8 or FT4
# at runtime via the Mode menu (Settings → Mode: FT8 | FT4). The choice is
# saved to Station.txt and takes effect on next boot.
#
# Set OFF to build a smaller FT8-only binary with no mode-switch menu item:
#   idf.py build -DENABLE_FT4=OFF
# ---------------------------------------------------------------------------
option(ENABLE_FT4 "Include FT4 protocol support and runtime mode switch" ON)

if(ENABLE_FT4)
    add_compile_definitions(ENABLE_FT4=1)
    message(STATUS "FT4 support: ON (runtime mode switch via menu)")
else()
    add_compile_definitions(ENABLE_FT4=0)
    message(STATUS "FT4 support: OFF (FT8-only build)")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mini_ft8)

# V2 UAC reference diagnostics: keep the managed component pristine and compile
# only a build-local copy with the bad-isoc message promoted from DEBUG to WARN.
idf_component_get_property(uac_lib espressif__usb_host_uac COMPONENT_LIB)
idf_component_get_property(uac_dir espressif__usb_host_uac COMPONENT_DIR)
set(uac_patch "${CMAKE_CURRENT_SOURCE_DIR}/patch_uac_diag.py")
set(uac_original "${uac_dir}/uac_host.c")
set(uac_patched "${CMAKE_BINARY_DIR}/v2_uac_diag/uac_host.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${uac_patch}" "${uac_original}")
execute_process(
    COMMAND "${PYTHON}" "${uac_patch}" "${uac_original}" "${uac_patched}"
    RESULT_VARIABLE uac_patch_result)
if(NOT uac_patch_result EQUAL 0)
    message(FATAL_ERROR "V2 UAC diagnostic patch failed")
endif()
get_target_property(uac_sources ${uac_lib} SOURCES)
set(uac_replaced FALSE)
set(uac_build_sources "")
foreach(source IN LISTS uac_sources)
    get_filename_component(source_name "${source}" NAME)
    if(source_name STREQUAL "uac_host.c")
        list(APPEND uac_build_sources "${uac_patched}")
        set(uac_replaced TRUE)
    else()
        list(APPEND uac_build_sources "${source}")
    endif()
endforeach()
if(NOT uac_replaced)
    message(FATAL_ERROR "V2 UAC component source layout changed")
endif()
set_property(TARGET ${uac_lib} PROPERTY SOURCES "${uac_build_sources}")
message(STATUS "V2 UAC diagnostic: bad isochronous packets logged at WARN")

# This release requires the full merged image because it installs the FATFS-only
# partition table. First boot formats the new 3 MB FATFS when needed.
add_custom_command(TARGET app POST_BUILD
    COMMAND ${PYTHON} -m esptool --chip esp32s3 merge_bin -o "MiniFT8_Merged_Auto.bin" @flash_args
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Auto-generating merged binary MiniFT8_Merged_Auto.bin for release..."
)
