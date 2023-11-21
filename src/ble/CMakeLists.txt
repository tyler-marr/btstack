# Note that headers are optional, and do not affect add_library, but they will
# not show up in IDEs unless they are listed in add_library.
set( SOURCE_LIST
    att_db.c
    att_db_util.c
    att_dispatch.c
    att_server.c
    gatt-service/ancs_client.c
    gatt-service/battery_service_client.c
    gatt-service/battery_service_server.c
    gatt-service/bond_management_service_server.c
    gatt-service/cycling_power_service_server.c
    gatt-service/cycling_speed_and_cadence_service_server.c
    gatt-service/device_information_service_client.c
    gatt-service/device_information_service_server.c
    gatt-service/heart_rate_service_server.c
    gatt-service/hids_client.c
    gatt-service/hids_device.c
    gatt-service/nordic_spp_service_server.c
    gatt-service/scan_parameters_service_client.c
    gatt-service/scan_parameters_service_server.c
    gatt-service/tx_power_service_server.c
    gatt-service/ublox_spp_service_server.c
    gatt_client.c
    sm.c
)
set( HEADER_LIST
    att_db.h
    att_db_util.h
    att_dispatch.h
    att_server.h
    core.h
    gatt-service/ancs_client.h
    gatt-service/battery_service_client.h
    gatt-service/battery_service_server.h
    gatt-service/bond_management_service_server.h
    gatt-service/cycling_power_service_server.h
    gatt-service/cycling_speed_and_cadence_service_server.h
    gatt-service/device_information_service_client.h
    gatt-service/device_information_service_server.h
    gatt-service/heart_rate_service_server.h
    gatt-service/hids_client.h
    gatt-service/hids_device.h
    gatt-service/nordic_spp_service_server.h
    gatt-service/scan_parameters_service_client.h
    gatt-service/scan_parameters_service_server.h
    gatt-service/tx_power_service_server.h
    gatt-service/ublox_spp_service_server.h
    gatt_client.h
    le_device_db.h
    sm.h
)

add_library(le_device_db_tlv le_device_db_tlv.c le_device_db_tlv.h)
target_include_directories(le_device_db_tlv PUBLIC .)
target_link_libraries(le_device_db_tlv PUBLIC ble)

add_library(le_device_db_memory le_device_db_memory.c)
target_include_directories(le_device_db_memory PUBLIC .)
target_link_libraries(le_device_db_memory PUBLIC ble)

add_library(ble ${SOURCE_LIST} ${HEADER_LIST})
target_include_directories(ble PUBLIC .)
target_include_directories(ble PUBLIC gatt-service)

target_link_libraries(ble PUBLIC btstack_config)
target_link_libraries(ble PUBLIC btstack_core)

# IDEs should put the headers in a nice place
source_group(
    TREE "${CMAKE_CURRENT_SOURCE_DIR}/"
    PREFIX "Bluetooth LE Header Files"
    FILES ${HEADER_LIST})



