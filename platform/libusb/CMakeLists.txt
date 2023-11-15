# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.

file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
#set(HEADER_LIST "${CMAKE_CURRENT_SOURCE_DIR}/include/modern/lib.hpp")

# Make an automatic library - will be static or dynamic based on user setting
add_library(platform_libusb ${SOURCE_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(platform_libusb PUBLIC .)

# pkgconfig required to link libusb                                                                                                       
find_package(PkgConfig REQUIRED)
# libusb                                                                                                                                  
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
target_include_directories(platform_libusb PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_directories(platform_libusb PRIVATE ${LIBUSB_LIBRARY_DIRS})
target_link_libraries(platform_libusb PRIVATE ${LIBUSB_LIBRARIES})
target_link_libraries(platform_libusb PRIVATE btstack)
# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Platform libusb Header Files"
    FILES ${HEADER_LIST})
