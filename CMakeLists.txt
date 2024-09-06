# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040_canbus CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(newton-rp2040-neopixel C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# NEOPIXEL
### ADD Neopixel Library START
add_library(pico_neopixel INTERFACE)
target_sources(pico_neopixel INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/Adafruit_NeoPixel.cpp
)
pico_generate_pio_header(pico_neopixel ${CMAKE_CURRENT_LIST_DIR}/ws2812byte.pio)
target_include_directories(pico_neopixel INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include # for our common lwipopts or any other standard includes, if required
)

target_link_libraries(pico_neopixel INTERFACE pico_stdlib pico_stdio hardware_pio pico_malloc pico_mem_ops pico_stdio_usb)


### ADD Neopixel Library END


if (TARGET tinyusb_device)

  # Add executable. Default name is the project name, version 0.1

  add_executable(newton-rp2040-neopixel newton-rp2040-neopixel.cpp PwmIn.cpp)



  pico_set_program_name(newton-rp2040-neopixel "newton-rp2040-neopixel")
  pico_set_program_version(newton-rp2040-neopixel "0.1")

  # Generate PIO header
  pico_generate_pio_header(newton-rp2040-neopixel ${CMAKE_CURRENT_LIST_DIR}/blink.pio)
  pico_generate_pio_header(newton-rp2040-neopixel ${CMAKE_CURRENT_LIST_DIR}/ppm.pio)
  pico_generate_pio_header(newton-rp2040-neopixel ${CMAKE_CURRENT_LIST_DIR}/PwmIn.pio)
  # Modify the below lines to enable/disable output over UART/USB
  # pico_enable_stdio_uart(newton-rp2040-neopixel 0)
  # pico_enable_stdio_usb(newton-rp2040-neopixel 0)
  # --> pico_stdio_usb 
  

  # Add the standard library to the build
  target_link_libraries(newton-rp2040-neopixel
          pico_stdlib)

  # Add the standard include files to the build
  target_include_directories(newton-rp2040-neopixel PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
    ${CMAKE_CURRENT_LIST_DIR}/include
  )

  # Add any user requested libraries
  target_link_libraries(newton-rp2040-neopixel 
          hardware_pio
          hardware_timer
          hardware_watchdog
          hardware_pwm
          hardware_gpio
          pico_neopixel
          )

  pico_add_extra_outputs(newton-rp2040-neopixel)

elseif(PICO_ON_DEVICE)
    message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()