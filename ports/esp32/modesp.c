/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Paul Sokolovsky
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>

#include "rom/gpio.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "drivers/dht/dht.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_https_ota/esp_https_ota.h"
#include "modesp.h"

STATIC mp_obj_t esp_osdebug(size_t n_args, const mp_obj_t *args) {
    esp_log_level_t level = LOG_LOCAL_LEVEL;
    if (n_args == 2) {
        level = mp_obj_get_int(args[1]);
    }
    if (args[0] == mp_const_none) {
        // Disable logging
        esp_log_level_set("*", ESP_LOG_ERROR);
    } else {
        // Enable logging at the given level
        // TODO args[0] should set the UART to which debug is sent
        esp_log_level_set("*", level);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp_osdebug_obj, 1, 2, esp_osdebug);

STATIC mp_obj_t esp_flash_read(mp_obj_t offset_in, mp_obj_t buf_in) {
    mp_int_t offset = mp_obj_get_int(offset_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    esp_err_t res = spi_flash_read(offset, bufinfo.buf, bufinfo.len);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(esp_flash_read_obj, esp_flash_read);

STATIC mp_obj_t esp_flash_write(mp_obj_t offset_in, mp_obj_t buf_in) {
    mp_int_t offset = mp_obj_get_int(offset_in);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);
    esp_err_t res = spi_flash_write(offset, bufinfo.buf, bufinfo.len);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(esp_flash_write_obj, esp_flash_write);

STATIC mp_obj_t esp_flash_erase(mp_obj_t sector_in) {
    mp_int_t sector = mp_obj_get_int(sector_in);
    esp_err_t res = spi_flash_erase_sector(sector);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_flash_erase_obj, esp_flash_erase);

STATIC mp_obj_t esp_flash_size(void) {
    return mp_obj_new_int_from_uint(spi_flash_get_chip_size());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(esp_flash_size_obj, esp_flash_size);

static const esp_partition_t* esp_partition_find_last(esp_partition_type_t type) {
    esp_partition_iterator_t iterator = esp_partition_find(type, ESP_PARTITION_SUBTYPE_ANY, NULL);
    const esp_partition_t *last = NULL;

    for (; NULL != iterator; iterator = esp_partition_next(iterator)) {
        last = esp_partition_get(iterator);
    }

    return last;
}

STATIC mp_obj_t esp_flash_user_start(void) {
    const esp_partition_t *last_app = esp_partition_find_last(ESP_PARTITION_TYPE_APP);
    const esp_partition_t *last_data = esp_partition_find_last(ESP_PARTITION_TYPE_DATA);

    uint32_t user_start = 0;

    if (last_app != NULL) {
        user_start = last_app->address + last_app->size;
    }

    if (last_data != NULL) {
        const uint32_t after_last_data = last_data->address + last_data->size;
        if (after_last_data > user_start) {
            user_start = after_last_data;
        }
    }

    if (user_start == 0) {
        mp_raise_OSError(MP_EIO);
    }

    return mp_obj_new_int(user_start);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(esp_flash_user_start_obj, esp_flash_user_start);

STATIC mp_obj_t esp_gpio_matrix_in(mp_obj_t pin, mp_obj_t sig, mp_obj_t inv) {
    gpio_matrix_in(mp_obj_get_int(pin), mp_obj_get_int(sig), mp_obj_get_int(inv));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_gpio_matrix_in_obj, esp_gpio_matrix_in);

STATIC mp_obj_t esp_gpio_matrix_out(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    gpio_matrix_out(mp_obj_get_int(args[0]), mp_obj_get_int(args[1]), mp_obj_get_int(args[2]), mp_obj_get_int(args[3]));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp_gpio_matrix_out_obj, 4, 4, esp_gpio_matrix_out);

STATIC mp_obj_t esp_neopixel_write_(mp_obj_t pin, mp_obj_t buf, mp_obj_t timing) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
    esp_neopixel_write(mp_hal_get_pin_obj(pin),
        (uint8_t*)bufinfo.buf, bufinfo.len, mp_obj_get_int(timing));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_neopixel_write_obj, esp_neopixel_write_);

/////////////////////
//OTA:
//typedef struct {
//    const char                  *url;                /*!< HTTP URL, the information on the URL is most important, it overrides the other fields below, if any */
//    const char                  *host;               /*!< Domain or IP as string */
//    int                         port;                /*!< Port to connect, default depend on esp_http_client_transport_t (80 or 443) */
//    const char                  *username;           /*!< Using for Http authentication */
//    const char                  *password;           /*!< Using for Http authentication */
//    esp_http_client_auth_type_t auth_type;           /*!< Http authentication type, see `esp_http_client_auth_type_t` */
//    const char                  *path;               /*!< HTTP Path, if not set, default is `/` */
//    const char                  *query;              /*!< HTTP query */
//    const char                  *cert_pem;           /*!< SSL Certification, PEM format as string, if the client requires to verify server */
//    esp_http_client_method_t    method;                   /*!< HTTP Method */
//    int                         timeout_ms;               /*!< Network timeout in milliseconds */
//    bool                        disable_auto_redirect;    /*!< Disable HTTP automatic redirects */
//    int                         max_redirection_count;    /*!< Max redirection number, using default value if zero*/
//    http_event_handle_cb        event_handler;             /*!< HTTP Event Handle */
//    esp_http_client_transport_t transport_type;           /*!< HTTP transport type, see `esp_http_client_transport_t` */
//    int                         buffer_size;              /*!< HTTP buffer size (both send and receive) */
//    void                        *user_data;               /*!< HTTP user_data context */
//} esp_http_client_config_t;
static const char* get_str_if_not_null(mp_obj_t o) {
    if (o == MP_OBJ_NULL) {
        return NULL;
    }

    return mp_obj_str_get_str(o);
}

static esp_err_t ota_http_event_handler(esp_http_client_event_t *evt)
{
    static const char *TAG = "ota_http_event_handler";
    
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

STATIC mp_obj_t mp_esp_https_ota(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_url, ARG_host, ARG_port, ARG_username, ARG_password,
        ARG_auth_type, ARG_path, ARG_query, ARG_cert_pem,
        ARG_method, ARG_timeout_ms, ARG_disable_auto_redirect,
        ARG_max_redirection_count, ARG_transport_type, ARG_buffer_size };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_url, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_host, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_port, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_username, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_password, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_auth_type, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_path, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_query, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_cert_pem, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_method, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_timeout_ms, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_disable_auto_redirect, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_max_redirection_count, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_transport_type, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_buffer_size, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    esp_http_client_config_t config = {
        .url = get_str_if_not_null(args[ARG_url].u_obj),
        .host = get_str_if_not_null(args[ARG_host].u_obj),
        .port = args[ARG_port].u_int,
        .username = get_str_if_not_null(args[ARG_username].u_obj),
        .password = get_str_if_not_null(args[ARG_password].u_obj),
        .auth_type = args[ARG_auth_type].u_int,
        .path = get_str_if_not_null(args[ARG_path].u_obj),
        .query = get_str_if_not_null(args[ARG_query].u_obj),
        .cert_pem = get_str_if_not_null(args[ARG_cert_pem].u_obj),
        .method = args[ARG_method].u_int,
        .timeout_ms = args[ARG_timeout_ms].u_int,
        .disable_auto_redirect = args[ARG_disable_auto_redirect].u_bool,
        .max_redirection_count = args[ARG_max_redirection_count].u_int,
        .transport_type = args[ARG_transport_type].u_int,
        .buffer_size = args[ARG_buffer_size].u_int,
        .event_handler = ota_http_event_handler
    };

    esp_err_t ret = esp_https_ota(&config);
    return ESP_OK == ret ? mp_const_true : mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(esp_https_ota_obj, 0, mp_esp_https_ota);

static mp_obj_t mp_obj_from_esp_partition_ptr(const esp_partition_t *partition) {
    return mp_obj_new_bytes((const byte *)&partition, sizeof(partition));
}

static const esp_partition_t* esp_partition_ptr_from_mp_obj(mp_obj_t partition_ptr_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(partition_ptr_in, &bufinfo, MP_BUFFER_READ);
    return *((esp_partition_t **)(bufinfo.buf));
}

static mp_obj_t mp_obj_from_esp_partition(const esp_partition_t *partition) {
    if (NULL == partition) {
        return mp_const_none;
    }

    const mp_obj_t partition_dict_map[][2] = {
        { MP_OBJ_NEW_QSTR(MP_QSTR_ptype), mp_obj_new_int(partition->type) },
        { MP_OBJ_NEW_QSTR(MP_QSTR_subtype), mp_obj_new_int(partition->subtype) },
        { MP_OBJ_NEW_QSTR(MP_QSTR_size), mp_obj_new_int(partition->size) },
        { MP_OBJ_NEW_QSTR(MP_QSTR_label),
          mp_obj_new_str(partition->label,
                         strnlen(partition->label, sizeof(partition->label) - 1)) }, 
        { MP_OBJ_NEW_QSTR(MP_QSTR_encrypted), mp_obj_new_bool(partition->encrypted) },
        { MP_OBJ_NEW_QSTR(MP_QSTR_handle), mp_obj_from_esp_partition_ptr(partition) },
    };

    const size_t partition_dict_size = sizeof(partition_dict_map) / sizeof(partition_dict_map[0]);
    mp_obj_t partition_dict = mp_obj_new_dict(partition_dict_size);

    for (int i = 0; i < partition_dict_size; ++i) {
        mp_obj_dict_store(partition_dict, partition_dict_map[i][0], partition_dict_map[i][1]);
    }

    return partition_dict; //mp_obj_new_tuple(sizeof(tuple) / sizeof(tuple[0]), tuple);
}

STATIC mp_obj_t mp_esp_ota_get_boot_partition(void) {
    return mp_obj_from_esp_partition(esp_ota_get_boot_partition());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(esp_ota_get_boot_partition_obj, mp_esp_ota_get_boot_partition);

STATIC mp_obj_t mp_esp_ota_get_running_partition(void) {
    return mp_obj_from_esp_partition(esp_ota_get_running_partition());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(esp_ota_get_running_partition_obj, mp_esp_ota_get_running_partition);

STATIC mp_obj_t mp_esp_ota_get_next_update_partition(mp_uint_t n_args, const mp_obj_t *args) {
    const esp_partition_t *start_from = NULL;
    if (n_args == 1) {
        start_from = esp_partition_ptr_from_mp_obj(args[0]);
    }
    
    return mp_obj_from_esp_partition(esp_ota_get_next_update_partition(start_from));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(esp_ota_get_next_update_partition_obj, 0, 1, mp_esp_ota_get_next_update_partition);

#if 0
static mp_obj_t mp_obj_from_esp_partition_iterator(esp_partition_iterator_t iterator) {
    return mp_obj_new_bytes(&iterator, sizeof(iterator));
}

static const esp_partition_iterator_t esp_partition_iterator_from_mp_obj(mp_obj_t partition_iterator_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(partition_iterator_in, &bufinfo, MP_BUFFER_READ);
    return *((esp_partition_iterator_t *)(bufinfo.buf));
}
#endif

STATIC mp_obj_t mp_esp_partition_get_ota_subtype(mp_obj_t ota_number_in) {
    const mp_int_t ota_number = ESP_PARTITION_SUBTYPE_APP_OTA_MIN + mp_obj_get_int(ota_number_in);

    if (ota_number > ESP_PARTITION_SUBTYPE_APP_OTA_MAX) {
        mp_raise_ValueError("OTA number out of range");
    }

    return mp_obj_new_int(ota_number);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_partition_get_ota_subtype_obj, mp_esp_partition_get_ota_subtype);

STATIC mp_obj_t mp_esp_partition_find(mp_obj_t type_in, mp_obj_t subtype_in, mp_obj_t label_in ) {
    const mp_int_t type = mp_obj_get_int(type_in);
    const mp_int_t subtype = mp_obj_get_int(subtype_in);
    const char* label = NULL;
    if (mp_obj_is_true(label_in)) {
        label = mp_obj_str_get_str(label_in);
    }
    
    mp_obj_t partition_list = mp_obj_new_list(0, NULL);
    esp_partition_iterator_t iterator = esp_partition_find(type, subtype, label);
    for (; iterator != NULL; iterator = esp_partition_next(iterator)) {
        const esp_partition_t *partition = esp_partition_get(iterator);
        mp_obj_list_append(partition_list, mp_obj_from_esp_partition(partition));
    }

    return partition_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_partition_find_obj, mp_esp_partition_find);

#if 0
STATIC mp_obj_t mp_esp_partition_iterator_release(mp_obj_t iterator_in) {
    esp_partition_iterator_t iterator = esp_partition_iterator_from_mp_obj(iterator_in);
    esp_partition_iterator_release(iterator);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_partition_iterator_release_obj, mp_esp_partition_iterator_release);

STATIC mp_obj_t mp_esp_partition_get(mp_obj_t iterator_in) {
    esp_partition_iterator_t iterator = esp_partition_iterator_from_mp_obj(iterator_in);
    return mp_obj_from_esp_partition(esp_partition_get(iterator));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_partition_get_obj, mp_esp_partition_get);

STATIC mp_obj_t mp_esp_partition_next(mp_obj_t iterator_in) {
    esp_partition_iterator_t iterator = esp_partition_iterator_from_mp_obj(iterator_in);
    return mp_obj_from_esp_partition_iterator(esp_partition_next(iterator));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_partition_next_obj, mp_esp_partition_next);
#endif

STATIC mp_obj_t mp_esp_partition_find_first(mp_obj_t type_in, mp_obj_t subtype_in, mp_obj_t label_in) {
    const mp_int_t type = mp_obj_get_int(type_in);
    const mp_int_t subtype = mp_obj_get_int(subtype_in);
    const char* label = NULL;
    if (mp_obj_is_true(label_in)) {
        label = mp_obj_str_get_str(label_in);
    }

    return mp_obj_from_esp_partition(esp_partition_find_first(type, subtype, label));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_partition_find_first_obj, mp_esp_partition_find_first);

STATIC mp_obj_t mp_esp_partition_read(mp_obj_t partition_in, mp_obj_t offset_in, mp_obj_t buf_in) {
    const mp_int_t offset = mp_obj_get_int(offset_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);

    const esp_partition_t *partition = esp_partition_ptr_from_mp_obj(partition_in);
    esp_err_t res = esp_partition_read(partition, offset, bufinfo.buf, bufinfo.len);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_partition_read_obj, mp_esp_partition_read);

STATIC mp_obj_t mp_esp_partition_write(mp_obj_t partition_in, mp_obj_t offset_in, mp_obj_t buf_in) {
    const mp_int_t offset = mp_obj_get_int(offset_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_READ);

    const esp_partition_t *partition = esp_partition_ptr_from_mp_obj(partition_in);
    esp_err_t res = esp_partition_write(partition, offset, bufinfo.buf, bufinfo.len);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_partition_write_obj, mp_esp_partition_write);

STATIC mp_obj_t mp_esp_partition_erase_range(mp_obj_t partition_in, mp_obj_t start_in, mp_obj_t size_in) {
    const mp_int_t start = mp_obj_get_int(start_in);
    const mp_int_t size = mp_obj_get_int(size_in);

    const esp_partition_t *partition = esp_partition_ptr_from_mp_obj(partition_in);
    esp_err_t res = esp_partition_erase_range(partition, start, size);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(esp_partition_erase_range_obj, mp_esp_partition_erase_range);
#if 0

 STATIC mp_obj_t mp_esp_ota_set_boot_partition (mp_obj_t partition_ptr_in) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(partition_ptr_in, &bufinfo, MP_BUFFER_READ);
    esp_partition_t* partition_ptr = NULL;
    memcpy(&partition_ptr, bufinfo.buf, bufinfo.len);
    esp_err_t res = esp_ota_set_boot_partition(partition_ptr);
    if (res != ESP_OK) {
        mp_raise_OSError(MP_EIO);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(esp_ota_set_boot_partition_obj, mp_esp_ota_set_boot_partition);

static const char *TAG = "OTA_UPDATE";
STATIC mp_obj_t esp_ota_set_boot(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    const mp_arg_t allowed_args[] = {
			{ MP_QSTR_ , MP_ARG_REQUIRED | MP_ARG_OBJ,  {.u_obj = mp_const_none} },
	};

	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);


    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Use partition label=%s", running->label);

    char *part_name = NULL;

    part_name = (char *)mp_obj_str_get_str(args[0].u_obj);

    ESP_LOGI(TAG, "Search partition label=%s", part_name);

    const esp_partition_t *boot_part1 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, part_name);
    const esp_partition_t *boot_part2 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, part_name);

    if ((boot_part1 == NULL) && (boot_part2 == NULL)) {
		ESP_LOGE(TAG, "Partition not found !");
		return mp_const_false;
    }

    // === Set boot partition ===
    char sptype[16] = {'\0'};
    char splabel[16] = {'\0'};
    esp_err_t err = ESP_FAIL;

    if (boot_part1 != NULL) {
    	sprintf(sptype,"Factory");
    	sprintf(splabel, boot_part1->label);
    	err = esp_ota_set_boot_partition(boot_part1);
    }
    else if (boot_part2 != NULL) {
    	sprintf(sptype,"OTA_0");
    	sprintf(splabel, boot_part2->label);
        const esp_partition_t *p = esp_ota_get_next_update_partition(boot_part2);
        ESP_LOGI(TAG, "Set partition label=%s",  p->label);
        err = esp_ota_set_boot_partition(boot_part2);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set_boot_partition failed! err=0x%x", err);
        return mp_const_false;
    }
    ESP_LOGW(TAG, "On next reboot the system will be started from '%s' partition (%s)", splabel, sptype);

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(esp_ota_set_boot_obj, 0, esp_ota_set_boot);
#endif

STATIC const mp_rom_map_elem_t esp_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_esp) },

    { MP_ROM_QSTR(MP_QSTR_osdebug), MP_ROM_PTR(&esp_osdebug_obj) },

    { MP_ROM_QSTR(MP_QSTR_flash_read), MP_ROM_PTR(&esp_flash_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_write), MP_ROM_PTR(&esp_flash_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_erase), MP_ROM_PTR(&esp_flash_erase_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_size), MP_ROM_PTR(&esp_flash_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_flash_user_start), MP_ROM_PTR(&esp_flash_user_start_obj) },

    { MP_ROM_QSTR(MP_QSTR_gpio_matrix_in), MP_ROM_PTR(&esp_gpio_matrix_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_gpio_matrix_out), MP_ROM_PTR(&esp_gpio_matrix_out_obj) },

    { MP_ROM_QSTR(MP_QSTR_neopixel_write), MP_ROM_PTR(&esp_neopixel_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_dht_readinto), MP_ROM_PTR(&dht_readinto_obj) },
    
    { MP_ROM_QSTR(MP_QSTR_https_ota), MP_ROM_PTR(&esp_https_ota_obj) },
    { MP_ROM_QSTR(MP_QSTR_ota_get_boot_partition), MP_ROM_PTR(&esp_ota_get_boot_partition_obj) },
    { MP_ROM_QSTR(MP_QSTR_ota_get_running_partition), MP_ROM_PTR(&esp_ota_get_running_partition_obj) },
    { MP_ROM_QSTR(MP_QSTR_ota_get_next_update_partition), MP_ROM_PTR(&esp_ota_get_next_update_partition_obj) },

    { MP_ROM_QSTR(MP_QSTR_partition_get_ota_subtype), MP_ROM_PTR(&esp_partition_get_ota_subtype_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_find_first), MP_ROM_PTR(&esp_partition_find_first_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_find), MP_ROM_PTR(&esp_partition_find_obj) },
#if 0
    { MP_ROM_QSTR(MP_QSTR_partition_get), MP_ROM_PTR(&esp_partition_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_next), MP_ROM_PTR(&esp_partition_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_iterator_release), MP_ROM_PTR(&esp_partition_iterator_release_obj) },
#endif
    { MP_ROM_QSTR(MP_QSTR_partition_read), MP_ROM_PTR(&esp_partition_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_write), MP_ROM_PTR(&esp_partition_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_partition_erase_range), MP_ROM_PTR(&esp_partition_erase_range_obj) },

#if 0
    { MP_ROM_QSTR(MP_QSTR_ota_set_boot_partition), MP_ROM_PTR(&esp_ota_set_boot_partition_obj) },
    
    { MP_ROM_QSTR(MP_QSTR_ota_set_bootpart),	MP_ROM_PTR(&esp_ota_set_boot_obj) },
#endif
    // Constant for flash
    { MP_ROM_QSTR(MP_QSTR_SPI_FLASH_SEC_SIZE), MP_ROM_INT(SPI_FLASH_SEC_SIZE) },

    // Constants for partition types and subtypes for find and find_first
    { MP_ROM_QSTR(MP_QSTR_PARTITION_TYPE_APP), MP_ROM_INT(ESP_PARTITION_TYPE_APP) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_TYPE_DATA), MP_ROM_INT(ESP_PARTITION_TYPE_DATA) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_APP_FACTORY), MP_ROM_INT(ESP_PARTITION_SUBTYPE_APP_FACTORY) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_APP_OTA_MIN), MP_ROM_INT(ESP_PARTITION_SUBTYPE_APP_OTA_MIN) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_APP_OTA_MAX), MP_ROM_INT(ESP_PARTITION_SUBTYPE_APP_OTA_MAX) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_APP_TEST), MP_ROM_INT(ESP_PARTITION_SUBTYPE_APP_TEST) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_OTA), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_OTA) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_PHY), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_PHY) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_NVS), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_NVS) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_COREDUMP), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_COREDUMP) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_ESPHTTPD), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_ESPHTTPD) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_FAT), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_FAT) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_DATA_SPIFFS), MP_ROM_INT(ESP_PARTITION_SUBTYPE_DATA_SPIFFS) },
    { MP_ROM_QSTR(MP_QSTR_PARTITION_SUBTYPE_ANY), MP_ROM_INT(ESP_PARTITION_SUBTYPE_ANY) },
    
    // Constants for second arg of osdebug()
    { MP_ROM_QSTR(MP_QSTR_LOG_NONE), MP_ROM_INT((mp_uint_t)ESP_LOG_NONE)},
    { MP_ROM_QSTR(MP_QSTR_LOG_ERROR), MP_ROM_INT((mp_uint_t)ESP_LOG_ERROR)},
    { MP_ROM_QSTR(MP_QSTR_LOG_WARNING), MP_ROM_INT((mp_uint_t)ESP_LOG_WARN)},
    { MP_ROM_QSTR(MP_QSTR_LOG_INFO), MP_ROM_INT((mp_uint_t)ESP_LOG_INFO)},
    { MP_ROM_QSTR(MP_QSTR_LOG_DEBUG), MP_ROM_INT((mp_uint_t)ESP_LOG_DEBUG)},
    { MP_ROM_QSTR(MP_QSTR_LOG_VERBOSE), MP_ROM_INT((mp_uint_t)ESP_LOG_VERBOSE)},
};

STATIC MP_DEFINE_CONST_DICT(esp_module_globals, esp_module_globals_table);

const mp_obj_module_t esp_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&esp_module_globals,
};

