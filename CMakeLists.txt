# Root CMakeLists.txt for ESP-IDF build system (used by matter_serial / matter_ota envs).
# Only active when framework = espidf, arduino is selected in platformio.ini.
# Regular arduino-only environments (serial_upload, OTA_upload) ignore this file.
cmake_minimum_required(VERSION 3.16.0)

# pioarduino registers src/ as an IDF component named "src", not "main".
# Tell esp_matter's CMakeLists.txt (which defaults to "main") to use "src" instead.
set(EXECUTABLE_COMPONENT_NAME "src")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(SolarControl)

# Exclude heavy components not needed for SolarControl
set(EXCLUDE_COMPONENTS "openthread" "esp_matter_console")

# C++17 required by Matter stack; CHIP_HAVE_CONFIG_H enables chip/CHIPConfig.h lookup
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS   "-Os" APPEND)
idf_build_set_property(COMPILE_OPTIONS     "-Wno-format-nonliteral;-Wno-format-security;-Wno-missing-field-initializers" APPEND)
