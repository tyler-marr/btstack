# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.
file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

add_library(platform_embedded ${SOURCE_LIST} ${HEADER_LIST})

target_include_directories(platform_embedded PUBLIC .)

target_link_libraries(platform_embedded PRIVATE btstack_config)
target_link_libraries(platform_embedded PRIVATE btstack_core)

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Platform embedded Header Files"
    FILES ${HEADER_LIST})
