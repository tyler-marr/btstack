# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.
set( SOURCE_LIST
    btstack_audio_portaudio.c
    btstack_link_key_db_fs.c
    btstack_network_posix.c
    btstack_run_loop_posix.c
    btstack_sco_transport_posix_i2s_test_bridge.c
    btstack_signal.c
    btstack_stdin_posix.c
    btstack_tlv_posix.c
    btstack_uart_posix.c
    hci_dump_posix_fs.c
    hci_dump_posix_stdout.c
    wav_util.c
)
set( HEADER_LIST
    btstack_link_key_db_fs.h
    btstack_run_loop_posix.h
    btstack_sco_transport_posix_i2s_test_bridge.h
    btstack_signal.h
    btstack_tlv_posix.h
    hci_dump_posix_fs.h
    hci_dump_posix_stdout.h
    wav_util.h
)

add_library(le_device_db_fs le_device_db_fs.c le_device_db_fs.h)
target_include_directories(le_device_db_fs PUBLIC .)
target_link_libraries(le_device_db_fs PRIVATE btstack_core)

add_library(platform_posix ${SOURCE_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(platform_posix PUBLIC .)

target_link_libraries(platform_posix PRIVATE btstack_core)
target_link_libraries(platform_posix PRIVATE btstack_config)
target_link_libraries(platform_posix PRIVATE tinydir)

# pkgconfig required to link libusb
find_package(PkgConfig REQUIRED)

# portaudio
pkg_check_modules(PORTAUDIO portaudio-2.0)
if(PORTAUDIO_FOUND)
    message("PortAudio found, enabling audio backend!")
    target_include_directories(platform_posix PRIVATE ${PORTAUDIO_INCLUDE_DIRS})
    target_link_directories(platform_posix PRIVATE ${PORTAUDIO_LIBRARY_DIRS})
    target_link_libraries(platform_posix PRIVATE ${PORTAUDIO_LIBRARIES})
    target_compile_definitions(platform_posix PUBLIC HAVE_PORTAUDIO)
endif()

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Platform Header Files"
    FILES ${HEADER_LIST})

