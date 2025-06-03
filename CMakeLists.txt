# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()

set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Paths to SDK and pico-extras
include(pico_sdk_import.cmake)

project(RAPID C CXX ASM)

# Initialise SDKs
pico_sdk_init()

# Add executable
add_executable(RAPID)

# Source files
target_sources(RAPID PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/RAPID.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/hw_config.c
)

# Program name and version
pico_set_program_name(RAPID "RAPID")
pico_set_program_version(RAPID "0.1")

add_subdirectory(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

add_compile_definitions(USE_PRINTF)

# Link libraries (add fatfs_sd from pico-extras)
target_link_libraries(RAPID PUBLIC
    pico_unique_id
    pico_stdlib
    tinyusb_device
    tinyusb_board
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

# Includes
target_include_directories(RAPID PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Build outputs
pico_add_extra_outputs(RAPID)
