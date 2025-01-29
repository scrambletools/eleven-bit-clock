# Install script for directory: C:/Users/john/Development/esp_fork/esp-idf

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/led_strip_spi_ws2812")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Users/john/Development/esp_tools/tools/riscv32-esp-elf/esp-14.2.0_20241119/riscv32-esp-elf/bin/riscv32-esp-elf-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/riscv/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/newlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_driver_rmt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/espressif__led_strip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Users/john/Development/pyramid_clock/eleven_bit_clock/managed_components/espressif__led_strip/examples/led_strip_spi_ws2812/build/esp-idf/main/cmake_install.cmake")
endif()

