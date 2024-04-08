/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "scan_after_connect.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"

static hci_con_handle_t connection_handle;
static gatt_client_service_t services[40];
static int service_count = 0;
static int service_index = 0;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void printUUID(uint8_t * uuid128, uint16_t uuid16){
    if (uuid16){
        printf("%04x",uuid16);
    } else {
        printf("%s", uuid128_to_str(uuid128));
    }
}

static void dump_characteristic(gatt_client_characteristic_t * characteristic){
    printf("    * characteristic: [0x%04x-0x%04x-0x%04x], properties 0x%02x, uuid ",
                            characteristic->start_handle, characteristic->value_handle, characteristic->end_handle, characteristic->properties);
    printUUID(characteristic->uuid128, characteristic->uuid16);
    printf("\n");
}

static void dump_service(gatt_client_service_t * service){
    printf("    * service: [0x%04x-0x%04x], uuid ", service->start_group_handle, service->end_group_handle);
    printUUID(service->uuid128, service->uuid16);
    printf("\n");
}

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            printf("BTstack activated, start scanning!\n");
            gap_set_scan_parameters(0,480, 48);
            gap_start_scan();
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (ad_data_contains_uuid16(gap_event_advertising_report_get_data_length(packet), gap_event_advertising_report_get_data(packet), 0xff10)){
                // stop scanning, and connect to the device
                bd_addr_t addr;
                gap_event_advertising_report_get_address(packet, addr);
                printf("GATT Counter found with %s, try to connect\n", bd_addr_to_str(addr));
                gap_stop_scan();
                gap_connect(addr, gap_event_advertising_report_get_address_type(packet));
            }
            break;
        case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
            if (ad_data_contains_uuid16(gap_event_extended_advertising_report_get_data_length(packet), gap_event_extended_advertising_report_get_data(packet), 0xff10)){
                // stop scanning, and connect to the device
                bd_addr_t addr;
                gap_event_extended_advertising_report_get_address(packet, addr);
                printf("GATT Counter found with %s, try to connect\n", bd_addr_to_str(addr));
                gap_stop_scan();
                gap_connect(addr, gap_event_extended_advertising_report_get_address_type(packet));
            }
            break;
        case HCI_EVENT_META_GAP:
            // wait for connection complete
            if (hci_event_gap_meta_get_subevent_code(packet) !=  GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            printf("\nGATT browser - CONNECTED\n");
            connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
            // query primary services
            gatt_client_discover_primary_services(handle_gatt_client_event, connection_handle);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("\nGATT browser - DISCONNECTED\n");
            break;
        default:
            break;
    }
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    gatt_client_service_t service;
    gatt_client_characteristic_t characteristic;
    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            gatt_event_service_query_result_get_service(packet, &service);
            dump_service(&service);
            services[service_count++] = service;
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            dump_characteristic(&characteristic);
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            // GATT_EVENT_QUERY_COMPLETE of search characteristics
            if (service_index < service_count) {
                service = services[service_index++];
                printf("\nGATT browser - CHARACTERISTIC for SERVICE %s, [0x%04x-0x%04x]\n",
                    uuid128_to_str(service.uuid128), service.start_group_handle, service.end_group_handle);
                gatt_client_discover_characteristics_for_service(handle_gatt_client_event, connection_handle, &service);
                break;
            }
            service_index = 0;
            // query complete, connect to next device
            printf("Query complete, connect to next device\n");
            gap_start_scan();
            break;
        default:
            break;
    }
}
int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){
    // Initialize L2CAP and register HCI event handler
    l2cap_init();

    // Setup security manager (needed for random address)
    sm_init();

    // Initialize GATT client
    gatt_client_init();

    // use random address
    gap_random_address_set_mode(GAP_RANDOM_ADDRESS_NON_RESOLVABLE);

    // register for HCI events
    hci_event_callback_registration.callback = &handle_hci_event;
    hci_add_event_handler(&hci_event_callback_registration);

    // turn on!
    hci_power_control(HCI_POWER_ON);
    return 0;
}


