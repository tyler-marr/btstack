/*
 * Copyright (C) 2017 BlueKitchen GmbH
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

#define BTSTACK_FILE__ "btstack_hid_parser.c"

#include <inttypes.h>
#include <string.h>

#include "btstack_hid_parser.h"
#include "btstack_util.h"
#include "btstack_debug.h"

// Not implemented:
// - Support for Push/Pop
// - Optional Pretty Print of HID Descripor
// - Support to query descriptort for contained usages, e.g. to detect keyboard or mouse

// #define HID_PARSER_PRETTY_PRINT

/*
 *  btstack_hid_parser.c
 */

#ifdef HID_PARSER_PRETTY_PRINT

static const char * type_names[] = {
    "Main",
    "Global",
    "Local",
    "Reserved"
};
static const char * main_tags[] = {
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "Input ",
    "Output",
    "Collection",
    "Feature",
    "End Collection",
    "Reserved",
    "Reserved",
    "Reserved"
};
static const char * global_tags[] = { 
    "Usage Page",
    "Logical Minimum",
    "Logical Maximum",
    "Physical Minimum",
    "Physical Maximum",
    "Unit Exponent",
    "Unit",
    "Report Size",
    "Report ID",
    "Report Count",
    "Push",
    "Pop",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved"
};
static const char * local_tags[] = {
    "Usage",
    "Usage Minimum",
    "Usage Maximum",
    "Designator Index",
    "Designator Minimum",
    "Designator Maximum",
    "String Index",
    "String Minimum",
    "String Maximum",
    "Delimiter",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved"
};
#endif

static void hid_pretty_print_item(btstack_hid_parser_t * parser, hid_descriptor_item_t * item){
#ifdef HID_PARSER_PRETTY_PRINT
    const char ** item_tag_table;
    switch ((TagType)item->item_type){
        case Main:
            item_tag_table = main_tags;
            break;
        case Global:
            item_tag_table = global_tags;
            break;
        case Local:
            item_tag_table = local_tags;
            break;
        default:
            item_tag_table = NULL;
            break;
    } 
    const char * item_tag_name = "Invalid";
    if (item_tag_table){
        item_tag_name = item_tag_table[item->item_tag];
    }
    log_info("%-15s (%-6s) // %02x 0x%0008x", item_tag_name, type_names[item->item_type], parser->descriptor[parser->descriptor_pos], item->item_value);
#else
    UNUSED(parser);
    UNUSED(item);
#endif
}

// parse descriptor item and read up to 32-bit bit value
bool btstack_hid_parse_descriptor_item(hid_descriptor_item_t * item, const uint8_t * hid_descriptor, uint16_t hid_descriptor_len){

    const int hid_item_sizes[] = { 0, 1, 2, 4 };

    // parse item header
    if (hid_descriptor_len < 1u) return false;
    uint16_t pos = 0;
    uint8_t item_header = hid_descriptor[pos++];
    item->data_size = hid_item_sizes[item_header & 0x03u];
    item->item_type = (item_header & 0x0cu) >> 2u;
    item->item_tag  = (item_header & 0xf0u) >> 4u;
    // long item
    if ((item->data_size == 2u) && (item->item_tag == 0x0fu) && (item->item_type == 3u)){
        if (hid_descriptor_len < 3u) return false;
        item->data_size = hid_descriptor[pos++];
        item->item_tag  = hid_descriptor[pos++];
    }
    item->item_size =  pos + item->data_size;
    item->item_value = 0;

    // read item value
    if (hid_descriptor_len < item->item_size) return false;
    if (item->data_size > 4u) return false;
    int i;
    int sgnd = (item->item_type == Global) && (item->item_tag > 0u) && (item->item_tag < 5u);
    int32_t value = 0;
    uint8_t latest_byte = 0;
    for (i=0;i<item->data_size;i++){
        latest_byte = hid_descriptor[pos++];
        value = (latest_byte << (8*i)) | value;
    }
    if (sgnd && (item->data_size > 0u)){
        if (latest_byte & 0x80u) {
            value -= 1u << (item->data_size*8u);
        }
    }
    item->item_value = value;
    return true;
}

static bool btstack_hid_main_item_tag_matches_report_type(MainItemTag tag, hid_report_type_t report_type){
    switch (tag){
        case Input:
            return report_type == HID_REPORT_TYPE_INPUT;
        case Output:
            return report_type == HID_REPORT_TYPE_OUTPUT;
        case Feature:
            return report_type == HID_REPORT_TYPE_FEATURE;
        default:
            return false;
    }
}

static void btstack_hid_handle_global_item(btstack_hid_parser_t * parser, hid_descriptor_item_t * item){
    switch((GlobalItemTag)item->item_tag){
        case UsagePage:
            parser->global_usage_page = item->item_value;
            break;
        case LogicalMinimum:
            parser->global_logical_minimum = item->item_value;
            break;
        case LogicalMaximum:
            parser->global_logical_maximum = item->item_value;
            break;
        case ReportSize:
            parser->global_report_size = item->item_value;
            break;
        case ReportID:
            parser->global_report_id = item->item_value;
            break;
        case ReportCount:
            parser->global_report_count = item->item_value;
            break;

        // TODO handle tags
        case PhysicalMinimum:
        case PhysicalMaximum:
        case UnitExponent:
        case Unit:
        case Push:
        case Pop:
            break;

        default:
            btstack_assert(false);
            break;
    }
}

static void hid_find_next_usage(btstack_hid_parser_t * parser){
    bool have_usage_min = false;
    bool have_usage_max = false;
    parser->usage_range = false;
    btstack_hid_descriptor_iterator_t iterator;
    btstack_hid_descriptor_iterator_init(&iterator, &parser->descriptor[parser->usage_pos], parser->descriptor_len - parser->usage_pos);
    while ((parser->available_usages == 0u) && btstack_hid_descriptor_iterator_has_more(&iterator) ){
        hid_descriptor_item_t usage_item = *btstack_hid_descriptor_iterator_get_item(&iterator);
        if ((usage_item.item_type == Global) && (usage_item.item_tag == UsagePage)){
            parser->usage_page = usage_item.item_value;
        }
        if (usage_item.item_type == Local){
            uint32_t usage_value = (usage_item.data_size > 2u) ? usage_item.item_value : ((parser->usage_page << 16u) | usage_item.item_value);
            switch (usage_item.item_tag){
                case Usage:
                    parser->available_usages = 1;
                    parser->usage_minimum = usage_value;
                    break;
                case UsageMinimum:
                    parser->usage_minimum = usage_value;
                    have_usage_min = true;
                    break;
                case UsageMaximum:
                    parser->usage_maximum = usage_value;
                    have_usage_max = true;
                    break;
                default:
                    break;
            }
            if (have_usage_min && have_usage_max){
                parser->available_usages = parser->usage_maximum - parser->usage_minimum + 1u;
                parser->usage_range = true;
                if (parser->available_usages < parser->required_usages){
                    log_debug("Usage Min - Usage Max [%04"PRIx32"..%04"PRIx32"] < Report Count %u", parser->usage_minimum & 0xffff, parser->usage_maximum & 0xffff, parser->required_usages);
                }
            }
        }
    }
    parser->usage_pos += iterator.descriptor_pos;
}

static void hid_process_item(btstack_hid_parser_t * parser, hid_descriptor_item_t * item){
    hid_pretty_print_item(parser, item);
    int valid_field = 0;
    uint16_t report_id_before;
    switch ((TagType)item->item_type){
        case Main:
            valid_field = btstack_hid_main_item_tag_matches_report_type((MainItemTag) item->item_tag,
                                                                        parser->report_type);
            break;
        case Global:
            report_id_before = parser->global_report_id;
            btstack_hid_handle_global_item(parser, item);
            // track record id for report handling
            if ((GlobalItemTag)item->item_tag == ReportID){
                if (parser->active_record && (report_id_before != item->item_value)){
                    parser->active_record = 0;
                }
            }
            break;
        case Local:
        case Reserved:
            break;
        default:
            btstack_assert(false);
            break;
    }
    if (!valid_field) return;

    // verify record id
    if (parser->global_report_id && !parser->active_record){
        if (parser->report[0] != parser->global_report_id){
            return;
        }
        parser->report_pos_in_bit += 8u;
    }
    parser->active_record = 1;
    // handle constant fields used for padding
    if (item->item_value & 1){
        int item_bits = parser->global_report_size * parser->global_report_count;
#ifdef HID_PARSER_PRETTY_PRINT
        log_info("- Skip %u constant bits", item_bits);
#endif
        parser->report_pos_in_bit += item_bits;
        return;
    }
    // Empty Item
    if (parser->global_report_count == 0u) return;
    // let's start
    parser->required_usages = parser->global_report_count;
}

static void btstack_hid_parser_find_next_usage(btstack_hid_parser_t * parser){
    while (btstack_hid_descriptor_iterator_has_more(&parser->descriptor_iterator)){
        parser->descriptor_item = * btstack_hid_descriptor_iterator_get_item(&parser->descriptor_iterator);
        hid_process_item(parser, &parser->descriptor_item);
        if (parser->required_usages){
            hid_find_next_usage(parser);
            if (parser->available_usages) {
                parser->state = BTSTACK_HID_PARSER_USAGES_AVAILABLE;
                return;
            } else {
                log_debug("no usages found");
                parser->state = BTSTACK_HID_PARSER_COMPLETE;
                return;
            }
        } else {
            if ((TagType) (&parser->descriptor_item)->item_type == Main) {
                // reset usage
                parser->usage_pos = parser->descriptor_iterator.descriptor_pos;
                parser->usage_page = parser->global_usage_page;
            }
        }
    }
    // end of descriptor
    parser->state = BTSTACK_HID_PARSER_COMPLETE;
}

// PUBLIC API

void btstack_hid_parser_init(btstack_hid_parser_t * parser, const uint8_t * hid_descriptor, uint16_t hid_descriptor_len, hid_report_type_t hid_report_type, const uint8_t * hid_report, uint16_t hid_report_len){

    memset(parser, 0, sizeof(btstack_hid_parser_t));

    parser->descriptor     = hid_descriptor;
    parser->descriptor_len = hid_descriptor_len;
    parser->report_type    = hid_report_type;
    parser->report         = hid_report;
    parser->report_len     = hid_report_len;
    parser->state          = BTSTACK_HID_PARSER_SCAN_FOR_REPORT_ITEM;

    btstack_hid_descriptor_iterator_init(&parser->descriptor_iterator, hid_descriptor, hid_descriptor_len);

    btstack_hid_parser_find_next_usage(parser);
}

bool btstack_hid_parser_has_more(btstack_hid_parser_t * parser){
    return parser->state == BTSTACK_HID_PARSER_USAGES_AVAILABLE;
}

void btstack_hid_parser_get_field(btstack_hid_parser_t * parser, uint16_t * usage_page, uint16_t * usage, int32_t * value){

    *usage_page = parser->usage_minimum >> 16;

    // read field (up to 32 bit unsigned, up to 31 bit signed - 32 bit signed behaviour is undefined) - check report len
    bool is_variable   = (parser->descriptor_item.item_value & 2) != 0;
    bool is_signed     = parser->global_logical_minimum < 0;
    int pos_start     = btstack_min(  parser->report_pos_in_bit >> 3, parser->report_len);
    int pos_end       = btstack_min( (parser->report_pos_in_bit + parser->global_report_size - 1u) >> 3u, parser->report_len);
    int bytes_to_read = pos_end - pos_start + 1;
    int i;
    uint32_t multi_byte_value = 0;
    for (i=0;i < bytes_to_read;i++){
        multi_byte_value |= parser->report[pos_start+i] << (i*8);
    }
    uint32_t unsigned_value = (multi_byte_value >> (parser->report_pos_in_bit & 0x07u)) & ((1u<<parser->global_report_size)-1u);
    // log_debug("bit pos %2u, report size %u, start %u, end %u, len %u;; unsigned value %08x", parser->report_pos_in_bit, parser->global_report_size, pos_start, pos_end, parser->report_len, unsigned_value);
    if (is_variable){
        *usage      = parser->usage_minimum & 0xffffu;
        if (is_signed && (unsigned_value & (1u<<(parser->global_report_size-1u)))){
            *value = unsigned_value - (1u<<parser->global_report_size);
        } else {
            *value = unsigned_value;
        }
    } else {
        *usage  = unsigned_value;
        *value  = 1;
    }
    parser->required_usages--;
    parser->report_pos_in_bit += parser->global_report_size;

    // next usage
    if (is_variable){
        parser->usage_minimum++;
        parser->available_usages--;
        if (parser->usage_range && (parser->usage_minimum > parser->usage_maximum)){
            // usage min - max range smaller than report count, ignore remaining bit in report
            log_debug("Ignoring %u items without Usage", parser->required_usages);
            parser->report_pos_in_bit += parser->global_report_size * parser->required_usages;
            parser->required_usages = 0;
        }
    } else {
        if (parser->required_usages == 0u){
            parser->available_usages = 0;
        }
    }
    if (parser->available_usages) {
        return;
    }
    if (parser->required_usages == 0u){
        parser->state = BTSTACK_HID_PARSER_SCAN_FOR_REPORT_ITEM;
        btstack_hid_parser_find_next_usage(parser);
    } else {
        hid_find_next_usage(parser);
        if (parser->available_usages == 0u) {
            parser->state = BTSTACK_HID_PARSER_COMPLETE;
        }
    }
}

void btstack_hid_descriptor_iterator_init(btstack_hid_descriptor_iterator_t * iterator, const uint8_t * hid_descriptor, uint16_t hid_descriptor_len){
    iterator->descriptor = hid_descriptor;
    iterator->descriptor_pos = 0;
    iterator->descriptor_len = hid_descriptor_len;
    iterator->item_ready = false;
    iterator->valid = true;
}

bool btstack_hid_descriptor_iterator_has_more(btstack_hid_descriptor_iterator_t * iterator){
    if ((iterator->item_ready == false) && (iterator->descriptor_len > iterator->descriptor_pos)){
        uint16_t  item_len = iterator->descriptor_len - iterator->descriptor_pos;
        const uint8_t *item_data = &iterator->descriptor[iterator->descriptor_pos];
        bool ok = btstack_hid_parse_descriptor_item(&iterator->descriptor_item, item_data, item_len);
        if (ok){
            iterator->item_ready = true;
        } else {
            iterator->valid = false;
        }
    }
    return iterator->item_ready;
}

const hid_descriptor_item_t * const btstack_hid_descriptor_iterator_get_item(btstack_hid_descriptor_iterator_t * iterator){
    iterator->descriptor_pos += iterator->descriptor_item.item_size;
    iterator->item_ready = false;
    return &iterator->descriptor_item;
}

bool btstack_hid_descriptor_iterator_valid(btstack_hid_descriptor_iterator_t * iterator){
    return iterator->valid;
}

int btstack_hid_get_report_size_for_id(int report_id, hid_report_type_t report_type, uint16_t hid_descriptor_len, const uint8_t * hid_descriptor) {
    int total_report_size = 0;
    int report_size = 0;
    int report_count = 0;
    int current_report_id = 0;

    btstack_hid_descriptor_iterator_t iterator;
    btstack_hid_descriptor_iterator_init(&iterator, hid_descriptor, hid_descriptor_len);
    while (btstack_hid_descriptor_iterator_has_more(&iterator)) {
        const hid_descriptor_item_t *const item = btstack_hid_descriptor_iterator_get_item(&iterator);
        int valid_report_type = 0;
        switch (item->item_type) {
            case Global:
                switch ((GlobalItemTag) item->item_tag) {
                    case ReportID:
                        current_report_id = item->item_value;
                        break;
                    case ReportCount:
                        report_count = item->item_value;
                        break;
                    case ReportSize:
                        report_size = item->item_value;
                        break;
                    default:
                        break;
                }
                break;
            case Main:
                if (current_report_id != report_id) break;
                switch ((MainItemTag) item->item_tag) {
                    case Input:
                        if (report_type != HID_REPORT_TYPE_INPUT) break;
                        valid_report_type = 1;
                        break;
                    case Output:
                        if (report_type != HID_REPORT_TYPE_OUTPUT) break;
                        valid_report_type = 1;
                        break;
                    case Feature:
                        if (report_type != HID_REPORT_TYPE_FEATURE) break;
                        valid_report_type = 1;
                        break;
                    default:
                        break;
                }
                if (!valid_report_type) break;
                total_report_size += report_count * report_size;
                break;
            default:
                break;
        }
        if (total_report_size > 0 && current_report_id != report_id) break;
    }

    if (btstack_hid_descriptor_iterator_valid(&iterator)){
        return (total_report_size + 7) / 8;
    } else {
        return 0;
    }
}

hid_report_id_status_t btstack_hid_id_valid(int report_id, uint16_t hid_descriptor_len, const uint8_t * hid_descriptor){
    int current_report_id = -1;
    btstack_hid_descriptor_iterator_t iterator;
    btstack_hid_descriptor_iterator_init(&iterator, hid_descriptor, hid_descriptor_len);
    while (btstack_hid_descriptor_iterator_has_more(&iterator)) {
        const hid_descriptor_item_t *const item = btstack_hid_descriptor_iterator_get_item(&iterator);
        switch (item->item_type){
            case Global:
                switch ((GlobalItemTag)item->item_tag){
                    case ReportID:
                        current_report_id = item->item_value;
                        if (current_report_id != report_id) break;
                        return HID_REPORT_ID_VALID;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }

    if (btstack_hid_descriptor_iterator_valid(&iterator)) {
        if (current_report_id != -1) {
            return HID_REPORT_ID_INVALID;
        } else {
            return HID_REPORT_ID_UNDECLARED;
        }
    } else {
        return HID_REPORT_ID_INVALID;
    }
}

bool btstack_hid_report_id_declared(uint16_t hid_descriptor_len, const uint8_t * hid_descriptor){
    btstack_hid_descriptor_iterator_t iterator;
    btstack_hid_descriptor_iterator_init(&iterator, hid_descriptor, hid_descriptor_len);
    while (btstack_hid_descriptor_iterator_has_more(&iterator)) {
        const hid_descriptor_item_t *const item = btstack_hid_descriptor_iterator_get_item(&iterator);
        switch (item->item_type){
            case Global:
                switch ((GlobalItemTag)item->item_tag){
                    case ReportID:
                        return true;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
    }
    return false;
}

static void btstack_parser_usage_iterator_process_item2(btstack_hid_parser_t * parser, hid_descriptor_item_t * item){
    hid_pretty_print_item(parser, item);
    int valid_field = 0;
    uint16_t report_id_before;
    switch ((TagType)item->item_type){
        case Main:
            valid_field = btstack_hid_main_item_tag_matches_report_type((MainItemTag) item->item_tag,
                                                                        parser->report_type);
            break;
        case Global:
            report_id_before = parser->global_report_id;
            btstack_hid_handle_global_item(parser, item);
            // track record id for report handling, reset report position
            if (report_id_before != parser->global_report_id){
                parser->report_pos_in_bit = 8u;
            }
            break;
        case Local:
        case Reserved:
            break;
        default:
            btstack_assert(false);
            break;
    }
    if (!valid_field) return;

    // handle constant fields used for padding
    if (item->item_value & 1){
        int item_bits = parser->global_report_size * parser->global_report_count;
#ifdef HID_PARSER_PRETTY_PRINT
        log_info("- Skip %u constant bits", item_bits);
#endif
        parser->report_pos_in_bit += item_bits;
        return;
    }
    // Empty Item
    if (parser->global_report_count == 0u) return;
    // let's start
    parser->required_usages = parser->global_report_count;
}

static void btstack_hid_usage_iterator_find_next_usage(btstack_hid_parser_t * parser) {
    while (btstack_hid_descriptor_iterator_has_more(&parser->descriptor_iterator)){
        parser->descriptor_item = * btstack_hid_descriptor_iterator_get_item(&parser->descriptor_iterator);

        btstack_parser_usage_iterator_process_item2(parser, &parser->descriptor_item);

        if (parser->required_usages){
            hid_find_next_usage(parser);
            if (parser->available_usages) {
                parser->state = BTSTACK_HID_PARSER_USAGES_AVAILABLE;
                return;
            } else {
                log_debug("no usages found");
                parser->state = BTSTACK_HID_PARSER_COMPLETE;
                return;
            }
        } else {
            if ((TagType) (&parser->descriptor_item)->item_type == Main) {
                // reset usage
                parser->usage_pos = parser->descriptor_iterator.descriptor_pos;
                parser->usage_page = parser->global_usage_page;
            }
        }
    }
    // end of descriptor
    parser->state = BTSTACK_HID_PARSER_COMPLETE;
}

void btstack_hid_usage_iterator_init(btstack_hid_parser_t * parser, const uint8_t * hid_descriptor, uint16_t hid_descriptor_len, hid_report_type_t hid_report_type){
    memset(parser, 0, sizeof(btstack_hid_parser_t));

    parser->descriptor     = hid_descriptor;
    parser->descriptor_len = hid_descriptor_len;
    parser->report_type    = hid_report_type;
    parser->state          = BTSTACK_HID_PARSER_SCAN_FOR_REPORT_ITEM;
    btstack_hid_descriptor_iterator_init(&parser->descriptor_iterator, hid_descriptor, hid_descriptor_len);

    btstack_hid_usage_iterator_find_next_usage(parser);
}

bool btstack_hid_usage_iterator_has_more(btstack_hid_parser_t * parser){
    return parser->state == BTSTACK_HID_PARSER_USAGES_AVAILABLE;
}

void btstack_hid_usage_iterator_get_item(btstack_hid_parser_t * parser, btstack_hid_usage_item_t * item){
    // cache current values
    memset(item, 0, sizeof(btstack_hid_usage_item_t));
    item->size = parser->global_report_size;
    item->report_id = parser->global_report_id;
    item->usage_page = parser->usage_minimum >> 16;
    item->bit_pos = parser->report_pos_in_bit;

    bool is_variable  = (parser->descriptor_item.item_value & 2) != 0;
    if (is_variable){
        item->usage = parser->usage_minimum & 0xffffu;
    }
    parser->required_usages--;
    parser->report_pos_in_bit += parser->global_report_size;

    // next usage
    if (is_variable){
        parser->usage_minimum++;
        parser->available_usages--;
        if (parser->usage_range && (parser->usage_minimum > parser->usage_maximum)){
            // usage min - max range smaller than report count, ignore remaining bit in report
            log_debug("Ignoring %u items without Usage", parser->required_usages);
            parser->report_pos_in_bit += parser->global_report_size * parser->required_usages;
            parser->required_usages = 0;
        }
    } else {
        if (parser->required_usages == 0u){
            parser->available_usages = 0;
        }
    }
    if (parser->available_usages) {
        return;
    }
    if (parser->required_usages == 0u){
        parser->state = BTSTACK_HID_PARSER_SCAN_FOR_REPORT_ITEM;
        btstack_hid_usage_iterator_find_next_usage(parser);
    } else {
        hid_find_next_usage(parser);
        if (parser->available_usages == 0u) {
            parser->state = BTSTACK_HID_PARSER_COMPLETE;
        }
    }
}

//