file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/srce/*.c")

# Make an automatic library - will be static or dynamic based on user setting
add_library(bluedroid_decoder ${SOURCE_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(bluedroid_decoder PUBLIC ./include)

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PREFIX "bluedroid decoder Header Files"
  FILES ${HEADER_LIST})

