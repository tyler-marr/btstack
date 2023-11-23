cmake_minimum_required(VERSION 3.17)
# Apple shenanigans
set(CMAKE_OSX_SYSROOT /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk)

project(
    BTstack
    VERSION 1.0
    DESCRIPTION "Dual-mode Bluetooth stack, with small memory footprint."
    LANGUAGES C)


set(BTSTACK_PORT unknown CACHE STRING "Use sepcified port for the examples to build with")
set_property(CACHE BTSTACK_PORT PROPERTY STRINGS libusb zephyr)

if(BTSTACK_PORT STREQUAL "unknown")
    get_property(BTSTACK_ALL_PORTS CACHE BTSTACK_PORT PROPERTY STRINGS)
    message( FATAL_ERROR "missing BTSTACK_PORT definition valid ports are: ${BTSTACK_ALL_PORTS}" )
endif()

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
  #find_package(Doxygen)
  #if(Doxygen_FOUND)
  #  add_subdirectory(docs)
  #else()
  #  message(STATUS "Doxygen not found, not building docs")
  #endif()
endif()

# In source builds cause a lot of problems and unneccessary complexity
function(prevent_in_source_builds)
    if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
        message(FATAL_ERROR "In-source builds are not allowed")
    endif()
endfunction()

prevent_in_source_builds()

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
if( BTSTACK_PORT STREQUAL "libusb")
    add_subdirectory(port/libusb)
elseif(BSTACK_PORT STREQUAL "zephy")
    add_subdirectory(port/zephyr)
endif()

add_subdirectory(src)
add_subdirectory(example)
add_subdirectory(platform/posix)
add_subdirectory(platform/libusb)
add_subdirectory(platform/embedded)
add_subdirectory(chipset/realtek)
add_subdirectory(chipset/zephyr)
add_subdirectory(3rd-party/rijndael)
add_subdirectory(3rd-party/micro-ecc)
add_subdirectory(3rd-party/lc3-google)
add_subdirectory(3rd-party/bluedroid/decoder)
add_subdirectory(3rd-party/bluedroid/encoder)
add_subdirectory(3rd-party/tinydir)
add_subdirectory(3rd-party/yxml)
add_subdirectory(3rd-party/md5)
add_subdirectory(3rd-party/hxcmod-player)

# Testing only available if this is the main app
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
