cmake_minimum_required(VERSION 3.19)

project(
  BTStackBase
  VERSION 1.0
  DESCRIPTION "Dual-mode Bluetooth stack, with small memory footprint."
  LANGUAGES C)

set(BTSTACK_PORT libusb CACHE STRING "Use sepcified port for the examples to build with")
set_property(CACHE BTSTACK_PORT PROPERTY STRINGS libusb zephyr)

# Only do these if this is the main project, and not if it is included through
# add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Optionally set things like CMAKE_CXX_STANDARD,
  # CMAKE_POSITION_INDEPENDENT_CODE here

  # Let's ensure -std=c++xx instead of -std=g++xx
  #set(CMAKE_CXX_EXTENSIONS OFF)

  # Let's nicely support folders in IDE's
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Testing only available if this is the main app. Note this needs to be done
  # in the main CMakeLists since it calls enable_testing, which must be in the
  # main CMakeLists.
  #include(CTest)

  # Docs only available if this is the main app
  find_package(Doxygen)
  if(Doxygen_FOUND)
    add_subdirectory(docs)
  else()
    message(STATUS "Doxygen not found, not building docs")
  endif()
endif()

# FetchContent added in CMake 3.11, downloads during the configure step
# FetchContent_MakeAvailable was not added until CMake 3.14
#include(FetchContent)

# Formatting library, adds fmt::fmt Always use the full git hash, not the tag,
# safer and faster to recompile
#FetchContent_Declare(
#  fmtlib
#  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
#  GIT_TAG 8.0.1)
#FetchContent_MakeAvailable(fmtlib)

# The compiled library code is here
add_subdirectory(src)
if( BTSTACK_PORT STREQUAL "libusb")
    add_subdirectory(port/libusb)
elseif(BSTACK_PORT STREQUAL "zephy")
    add_subdirectory(port/zephyr)
endif()
add_subdirectory(example)
add_subdirectory(platform/posix)
add_subdirectory(platform/libusb)
add_subdirectory(chipset/realtek)
add_subdirectory(chipset/zephyr)
add_subdirectory(3rd-party/rijndael)
add_subdirectory(3rd-party/micro-ecc)
add_subdirectory(3rd-party/lc3-google)
add_subdirectory(3rd-party/bluedroid/decoder)
add_subdirectory(3rd-party/bluedroid/encoder)
add_subdirectory(3rd-party/tinydir)
add_subdirectory(3rd-party/yxml)

# Testing only available if this is the main app
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
