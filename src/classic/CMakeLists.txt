# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.

file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
#set(HEADER_LIST "${ModernCMakeExample_SOURCE_DIR}/include/modern/lib.hpp")

# Make an automatic library - will be static or dynamic based on user setting
add_library(classic ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(classic PUBLIC .)
target_link_libraries(classic PRIVATE btstack)
target_link_libraries(classic PRIVATE bluedroid_decoder)
target_link_libraries(classic PRIVATE bluedroid_encoder)
target_link_libraries(classic PRIVATE yxml)

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Bluetooth Classic Header Files"
    FILES ${HEADER_LIST})

