# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.

file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.c")
#set(HEADER_LIST "${ModernCMakeExample_SOURCE_DIR}/include/modern/lib.hpp")

# Make an automatic library - will be static or dynamic based on user setting
add_library(btstack ${SOURCE_LIST} ${HEADER_LIST})

target_compile_options(btstack PRIVATE
     $<$<OR:$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>>:
        -Wunused-variable -Wswitch-default -Werror>
     $<$<C_COMPILER_ID:GNU>:
     -Wall -Wextra -Wunused-but-set-variable -Wunused-variable -Wswitch-default -Werror>
     $<$<C_COMPILER_ID:MSVC>:
        /WX /W4>)

target_link_libraries( btstack PUBLIC port )
target_link_libraries( btstack PRIVATE rijndael )
target_link_libraries( btstack PRIVATE uecc )
target_link_libraries( btstack PRIVATE lc3-google )
target_link_libraries( btstack PUBLIC ble )
target_link_libraries( btstack PUBLIC gatt_service )
target_link_libraries( btstack PUBLIC classic )

# We need this directory, and users of our library will need it too
target_include_directories( btstack PUBLIC . )
add_subdirectory( ble )
add_subdirectory( classic )

target_compile_features( btstack PRIVATE c_std_99 )

# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/src"
  PREFIX "btstack Header Files"
  FILES ${HEADER_LIST})

