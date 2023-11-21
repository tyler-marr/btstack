# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.

#file(GLOB HEADER_LIST
#    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(btstack_config INTERFACE)
target_include_directories(btstack_config INTERFACE .)

add_library(port main.c)
target_include_directories(port PRIVATE .)
target_link_libraries(port PRIVATE le_device_db_tlv)
target_link_libraries(port PUBLIC btstack_config)
target_link_libraries(port PUBLIC platform_posix)
target_link_libraries(port PRIVATE platform_libusb)
target_link_libraries(port PUBLIC platform_embedded)
target_link_libraries(port PUBLIC btstack_core)
target_link_libraries(port PRIVATE chipset_realtek)
target_link_libraries(port PRIVATE chipset_zephyr)

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Port Header Files"
    FILES ${HEADER_LIST})
