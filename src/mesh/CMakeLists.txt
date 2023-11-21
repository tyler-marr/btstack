# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.
set( SOURCE_LIST
    adv_bearer.c
    beacon.c
    gatt-service/mesh_provisioning_service_server.c
    gatt-service/mesh_proxy_service_server.c
    gatt_bearer.c
    mesh.c
    mesh_access.c
    mesh_configuration_client.c
    mesh_configuration_server.c
    mesh_crypto.c
    mesh_foundation.c
    mesh_generic_default_transition_time_client.c
    mesh_generic_default_transition_time_server.c
    mesh_generic_level_client.c
    mesh_generic_level_server.c
    mesh_generic_on_off_client.c
    mesh_generic_on_off_server.c
    mesh_health_server.c
    mesh_iv_index_seq_number.c
    mesh_keys.c
    mesh_lower_transport.c
    mesh_network.c
    mesh_node.c
    mesh_peer.c
    mesh_proxy.c
    mesh_upper_transport.c
    mesh_virtual_addresses.c
    pb_adv.c
    pb_gatt.c
    provisioning.c
    provisioning_device.c
    provisioning_provisioner.c
)
set( HEADER_LIST
    adv_bearer.h
    beacon.h
    gatt-service/mesh_provisioning_service_server.h
    gatt-service/mesh_proxy_service_server.h
    gatt_bearer.h
    mesh.h
    mesh_access.h
    mesh_configuration_client.h
    mesh_configuration_server.h
    mesh_crypto.h
    mesh_foundation.h
    mesh_generic_default_transition_time_client.h
    mesh_generic_default_transition_time_server.h
    mesh_generic_level_client.h
    mesh_generic_level_server.h
    mesh_generic_model.h
    mesh_generic_on_off_client.h
    mesh_generic_on_off_server.h
    mesh_health_server.h
    mesh_iv_index_seq_number.h
    mesh_keys.h
    mesh_lower_transport.h
    mesh_network.h
    mesh_node.h
    mesh_peer.h
    mesh_proxy.h
    mesh_upper_transport.h
    mesh_virtual_addresses.h
    pb_adv.h
    pb_gatt.h
    provisioning.h
    provisioning_device.h
    provisioning_provisioner.h
)
add_library(mesh ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories( mesh PUBLIC . )
target_link_libraries( mesh PRIVATE btstack_config )
target_link_libraries( mesh PRIVATE btstack_core )

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Mesh Header Files"
    FILES ${HEADER_LIST})


