# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.

file(GLOB HEADER_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB SOURCE_LIST
    CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")

# Make an automatic library - will be static or dynamic based on user setting
add_library(btstack_core ${SOURCE_LIST} ${HEADER_LIST})

target_compile_options(btstack_core PRIVATE
     $<$<OR:$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>>:
        -Wunused-variable -Wswitch-default -Werror>
     $<$<C_COMPILER_ID:GNU>:
     -Wall -Wextra -Wunused-but-set-variable -Wunused-variable -Wswitch-default -Werror>
     $<$<C_COMPILER_ID:MSVC>:
        /WX /W4>)

target_link_libraries( btstack_core PRIVATE btstack_config )
target_link_libraries( btstack_core PRIVATE rijndael )
target_link_libraries( btstack_core PRIVATE uecc )
target_link_libraries( btstack_core PRIVATE lc3-google )
target_link_libraries( btstack_core PUBLIC ble )
target_link_libraries( btstack_core PUBLIC mesh )
target_link_libraries( btstack_core PUBLIC classic )

# We need this directory, and users of our library will need it too
target_include_directories( btstack_core PUBLIC . )
target_compile_features( btstack_core PRIVATE c_std_99 )

add_subdirectory( ble )
add_subdirectory( classic )
add_subdirectory( mesh )

# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/src"
  PREFIX "BTstack Core Header Files"
  FILES ${HEADER_LIST})

