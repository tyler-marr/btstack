find_package (Python REQUIRED COMPONENTS Interpreter)

function(gatt_to_header out_var)
  set(result)
  foreach(in_f ${ARGN})
    file(RELATIVE_PATH out_f ${CMAKE_CURRENT_SOURCE_DIR} ${in_f})
    set(out_f "${CMAKE_CURRENT_BINARY_DIR}/${out_f}")
    string(REGEX REPLACE "\\.[^.]*$" ".h" out_f ${out_f})
    add_custom_command(OUTPUT ${out_f}
        COMMAND ${Python_EXECUTABLE}
        ARGS ${BTstack_SOURCE_DIR}/tool/compile_gatt.py ${in_f} ${out_f}
        DEPENDS ${in_f}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Creating gatt header file ${out_f}"
        VERBATIM
      )
    list(APPEND result ${out_f})
  endforeach()
  set(${out_var} "${result}" PARENT_SCOPE)
endfunction()


set( EXAMPLE_LIST
    a2dp_sink_demo.c
    a2dp_source_demo.c
    ancs_client_demo.c
    #ant_test.c
    att_delayed_response.c
    audio_duplex.c
    #avrcp_browsing_client.c
    dut_mode_classic.c
    gap_dedicated_bonding.c
    gap_inquiry.c
    gap_le_advertisements.c
    gap_link_keys.c
    gatt_battery_query.c
    gatt_browser.c
    gatt_counter.c
    gatt_device_information_query.c
    gatt_heart_rate_client.c
    gatt_streamer_server.c
    hfp_ag_demo.c
    hfp_hf_demo.c
    hid_host_demo.c
    hid_keyboard_demo.c
    hid_mouse_demo.c
    hog_boot_host_demo.c
    hog_host_demo.c
    hog_keyboard_demo.c
    hog_mouse_demo.c
    hsp_ag_demo.c
    hsp_hs_demo.c
    #le_audio_demo_util_sink.c
    #le_audio_demo_util_source.c
    le_credit_based_flow_control_mode_client.c
    le_credit_based_flow_control_mode_server.c
    le_mitm.c
    le_streamer_client.c
    led_counter.c
    mesh_node_demo.c
    mod_player.c
    nordic_spp_le_counter.c
    nordic_spp_le_streamer.c
    #pan_lwip_http_server.c
    panu_demo.c
    pbap_client_demo.c
    #sco_demo_util.c
    sdp_bnep_query.c
    sdp_general_query.c
    sdp_rfcomm_query.c
    sine_player.c
    sm_pairing_central.c
    sm_pairing_peripheral.c
    spp_and_gatt_counter.c
    spp_and_gatt_streamer.c
    spp_counter.c
    spp_flowcontrol.c
    spp_streamer.c
    spp_streamer_client.c
    ublox_spp_le_counter.c
)

file(GLOB GATT_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.gatt")
gatt_to_header( gatt_headers ${GATT_FILES} )

# create targets
foreach(EXAMPLE_FILE ${EXAMPLE_LIST})
	get_filename_component(EXAMPLE ${EXAMPLE_FILE} NAME_WE)

    set( EXAMPLE_GATT_HEADER )
    if ( "${gatt_headers}" MATCHES "${EXAMPLE}\.h" )
        set( EXAMPLE_GATT_HEADER "${CMAKE_MATCH_0}" )
    endif()
    add_executable( ${EXAMPLE} ${EXAMPLE_FILE} ${EXAMPLE_GATT_HEADER})

    target_link_options(${EXAMPLE} PRIVATE LINKER:
        $<$<C_COMPILER_ID:AppleClang>:
        -no_warn_duplicate_libraries>)

    target_link_libraries( ${EXAMPLE} PRIVATE port )
    target_include_directories( ${EXAMPLE} PRIVATE ${CMAKE_CURRENT_BINARY_DIR} )
endforeach(EXAMPLE_FILE)


target_link_libraries( a2dp_source_demo PRIVATE hxcmod-player )
target_link_libraries( mod_player PRIVATE hxcmod-player )
target_sources( hfp_ag_demo PRIVATE sco_demo_util.c )
target_sources( hfp_hf_demo PRIVATE sco_demo_util.c )
target_sources( hsp_ag_demo PRIVATE sco_demo_util.c )
target_sources( hsp_hs_demo PRIVATE sco_demo_util.c )
