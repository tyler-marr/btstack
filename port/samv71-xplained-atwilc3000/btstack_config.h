// 
// btstack_config.h for SAMV71 + ATWILC3000
//

#ifndef __BTSTACK_CONFIG
#define __BTSTACK_CONFIG

#define HAVE_INIT_SCRIPT
#define HAVE_EMBEDDED_TICK
#define HAVE_UART_DMA_SET_FLOWCONTROL
#define HAVE_BTSTACK_STDIN

#define ENABLE_BLE
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_CENTRAL
#define ENABLE_LE_DATA_CHANNELS

#define HCI_ACL_PAYLOAD_SIZE 300

// memory config

#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_GATT_SUBCLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1
#define MAX_NR_RFCOMM_CHANNELS 0
#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_SERVICE_RECORD_ITEMS 0
#define MAX_NR_SM_LOOKUP_ENTRIES 1
#define MAX_NR_WHITELIST_ENTRIES 1

#endif

