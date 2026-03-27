# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    $ENV{ADF_PATH}/components/audio_pipeline
    $ENV{ADF_PATH}/components/audio_stream
    $ENV{ADF_PATH}/components/audio_hal
    $ENV{ADF_PATH}/components/audio_board
    $ENV{ADF_PATH}/components/audio_recorder
    $ENV{ADF_PATH}/components/esp-sr
    $ENV{ADF_PATH}/components/esp_codec_dev
    $ENV{ADF_PATH}/components/esp_peripherals
    $ENV{ADF_PATH}/components/audio_sal
    $ENV{ADF_PATH}/components/adf_utils
    components
)

include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(voice_assistant)
