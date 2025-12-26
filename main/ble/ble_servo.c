/**
 * @file ble_servo.c
 * @brief Minimal BLE Servo Controller Implementation
 * 
 * Uses NimBLE for Web Bluetooth compatible servo control.
 */

#include "ble_servo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "BLE_SERVO";

#if CONFIG_BT_NIMBLE_ENABLED

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// UUIDs for Web Bluetooth
// Service: 0d9be2a0-4757-43d9-83df-704ae274b8df
// Char:    8116d8c0-d45d-4fdf-998e-33ab8c471d59
#define SERVICE_UUID_128 \
    0xdf, 0xb8, 0x74, 0xe2, 0x4a, 0x70, 0xdf, 0x83, \
    0xd9, 0x43, 0x57, 0x47, 0xa0, 0xe2, 0x9b, 0x0d

#define CHAR_UUID_128 \
    0x59, 0x1d, 0x47, 0x8c, 0xab, 0x33, 0x8e, 0x99, \
    0xdf, 0x4f, 0x5d, 0xd4, 0xc0, 0xd8, 0x16, 0x81

static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(SERVICE_UUID_128);
static const ble_uuid128_t chr_uuid = BLE_UUID128_INIT(CHAR_UUID_128);

// State
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_chr_handle;
static bool s_connected = false;

// Chunked message buffer
#define CHUNK_BUFFER_SIZE 2048
static char s_chunk_buffer[CHUNK_BUFFER_SIZE];
static size_t s_chunk_len = 0;
static uint8_t s_chunk_expected = 0;
static uint8_t s_chunk_received = 0;

// Callbacks
static ble_servo_move_cb_t s_move_cb = NULL;
static ble_servo_stance_cb_t s_stance_cb = NULL;
static ble_servo_connect_cb_t s_connect_cb = NULL;

// Forward declarations
static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gap_event_cb(struct ble_gap_event *event, void *arg);
static void on_sync(void);
static void on_reset(int reason);
static void host_task(void *param);
static void start_advertising(void);
static void process_command(const char* cmd, size_t len);

// ═══════════════════════════════════════════════════════
// CHUNKED MESSAGE HANDLING
// ═══════════════════════════════════════════════════════

/**
 * Reset chunk buffer state
 */
static void chunk_reset(void) {
    s_chunk_len = 0;
    s_chunk_expected = 0;
    s_chunk_received = 0;
    s_chunk_buffer[0] = '\0';
}

/**
 * Handle incoming data - either chunked or regular
 * Chunk format: {"k":<num>,"t":<total>,"d":"<data>"}
 * When all chunks received, concatenates and processes as single command
 */
static void handle_incoming_data(const char* data, size_t len) {
    // Try to parse as chunk header
    cJSON* json = cJSON_ParseWithLength(data, len);
    if (!json) {
        ESP_LOGW(TAG, "Invalid JSON");
        return;
    }
    
    cJSON* k = cJSON_GetObjectItem(json, "k");  // chunk number (1-based)
    cJSON* t = cJSON_GetObjectItem(json, "t");  // total chunks
    cJSON* d = cJSON_GetObjectItem(json, "d");  // data payload
    
    // Check if this is a chunked message
    if (k && t && d && cJSON_IsNumber(k) && cJSON_IsNumber(t) && cJSON_IsString(d)) {
        uint8_t chunk_num = (uint8_t)k->valueint;
        uint8_t total = (uint8_t)t->valueint;
        const char* payload = d->valuestring;
        size_t payload_len = strlen(payload);
        
        ESP_LOGI(TAG, "Chunk %d/%d (%zu bytes)", chunk_num, total, payload_len);
        
        // First chunk - reset buffer
        if (chunk_num == 1) {
            chunk_reset();
            s_chunk_expected = total;
        }
        
        // Validate sequence
        if (chunk_num != s_chunk_received + 1 || total != s_chunk_expected) {
            ESP_LOGW(TAG, "Chunk sequence error, resetting");
            chunk_reset();
            ble_servo_send_response("{\"err\":\"chunk_seq\"}");
            cJSON_Delete(json);
            return;
        }
        
        // Append to buffer if there's room
        if (s_chunk_len + payload_len < CHUNK_BUFFER_SIZE - 1) {
            memcpy(s_chunk_buffer + s_chunk_len, payload, payload_len);
            s_chunk_len += payload_len;
            s_chunk_buffer[s_chunk_len] = '\0';
            s_chunk_received = chunk_num;
            
            // Send ack for this chunk
            char ack[32];
            snprintf(ack, sizeof(ack), "{\"ack\":%d}", chunk_num);
            ble_servo_send_response(ack);
            
            // All chunks received - process complete message
            if (s_chunk_received == s_chunk_expected) {
                ESP_LOGI(TAG, "All chunks received, total %zu bytes", s_chunk_len);
                process_command(s_chunk_buffer, s_chunk_len);
                chunk_reset();
            }
        } else {
            ESP_LOGE(TAG, "Chunk buffer overflow");
            chunk_reset();
            ble_servo_send_response("{\"err\":\"overflow\"}");
        }
        
        cJSON_Delete(json);
        return;
    }
    
    // Not a chunk - process as regular command
    cJSON_Delete(json);
    process_command(data, len);
}

// ═══════════════════════════════════════════════════════
// COMMAND PROCESSING
// ═══════════════════════════════════════════════════════

static void process_move_array(cJSON* arr) {
    if (!cJSON_IsArray(arr) || cJSON_GetArraySize(arr) < 5) return;
    
    float fr = (float)cJSON_GetArrayItem(arr, 0)->valuedouble;
    float fl = (float)cJSON_GetArrayItem(arr, 1)->valuedouble;
    float br = (float)cJSON_GetArrayItem(arr, 2)->valuedouble;
    float bl = (float)cJSON_GetArrayItem(arr, 3)->valuedouble;
    uint16_t speed = (uint16_t)cJSON_GetArrayItem(arr, 4)->valueint;
    uint16_t delay_ms = 0;
    if (cJSON_GetArraySize(arr) > 5) {
        delay_ms = (uint16_t)cJSON_GetArrayItem(arr, 5)->valueint;
    }
    
    ESP_LOGI(TAG, "Move: FR=%.0f FL=%.0f BR=%.0f BL=%.0f spd=%u dly=%u",
             fr, fl, br, bl, speed, delay_ms);
    
    if (s_move_cb) {
        s_move_cb(fr, fl, br, bl, speed, delay_ms);
    }
    
    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void process_command(const char* cmd, size_t len) {
    ESP_LOGI(TAG, "Cmd: %.*s", (int)len, cmd);
    
    cJSON* json = cJSON_ParseWithLength(cmd, len);
    if (!json) {
        ESP_LOGW(TAG, "Invalid JSON");
        return;
    }
    
    // Single move: {"s":[fr,fl,br,bl,speed,delay]}
    cJSON* s = cJSON_GetObjectItem(json, "s");
    if (s && cJSON_IsArray(s)) {
        process_move_array(s);
        cJSON_Delete(json);
        return;
    }
    
    // Sequence: {"m":[[fr,fl,br,bl,speed,delay], ...]}
    cJSON* m = cJSON_GetObjectItem(json, "m");
    if (m && cJSON_IsArray(m)) {
        int count = cJSON_GetArraySize(m);
        ESP_LOGI(TAG, "Sequence: %d moves", count);
        for (int i = 0; i < count; i++) {
            cJSON* move = cJSON_GetArrayItem(m, i);
            if (cJSON_IsArray(move)) {
                process_move_array(move);
            }
        }
        ble_servo_send_response("{\"ok\":1}");
        cJSON_Delete(json);
        return;
    }
    
    // Ping: {"p":1}
    cJSON* p = cJSON_GetObjectItem(json, "p");
    if (p) {
        ble_servo_send_response("{\"p\":1}");
        cJSON_Delete(json);
        return;
    }
    
    // Stance: {"r":1}
    cJSON* r = cJSON_GetObjectItem(json, "r");
    if (r) {
        ESP_LOGI(TAG, "Return to stance");
        if (s_stance_cb) {
            s_stance_cb();
        }
        ble_servo_send_response("{\"ok\":1}");
        cJSON_Delete(json);
        return;
    }
    
    ESP_LOGW(TAG, "Unknown command");
    cJSON_Delete(json);
}

// ═══════════════════════════════════════════════════════
// GATT CALLBACKS
// ═══════════════════════════════════════════════════════

static int chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (ctxt->om && ctxt->om->om_len > 0) {
            char* buf = malloc(ctxt->om->om_len + 1);
            if (buf) {
                ble_hs_mbuf_to_flat(ctxt->om, buf, ctxt->om->om_len, NULL);
                buf[ctxt->om->om_len] = '\0';
                handle_incoming_data(buf, ctxt->om->om_len);
                free(buf);
            }
        }
    }
    return 0;
}

// GATT service definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &chr_uuid.u,
            .access_cb = chr_access_cb,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &s_chr_handle,
        }, { 0 } }
    },
    { 0 },
};

// ═══════════════════════════════════════════════════════
// GAP / ADVERTISING
// ═══════════════════════════════════════════════════════

static void start_advertising(void) {
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = 0;
    fields.name = (uint8_t*)ble_svc_gap_device_name();
    fields.name_len = strlen((char*)fields.name);
    fields.name_is_complete = 1;
    
    ble_gap_adv_set_fields(&fields);
    
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_cb, NULL);
    
    ESP_LOGI(TAG, "Advertising as '%s'", ble_svc_gap_device_name());
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            ESP_LOGI(TAG, "Connected");
            if (s_connect_cb) s_connect_cb(true);
        } else {
            start_advertising();
        }
        break;
        
    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_connected = false;
        ESP_LOGI(TAG, "Disconnected");
        if (s_connect_cb) s_connect_cb(false);
        start_advertising();
        break;
        
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    }
    return 0;
}

static void on_sync(void) {
    ble_hs_util_ensure_addr(0);
    
    uint8_t addr[6];
    ble_hs_id_copy_addr(BLE_OWN_ADDR_PUBLIC, addr, NULL);
    ESP_LOGI(TAG, "BLE Addr: %02x:%02x:%02x:%02x:%02x:%02x",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
    
    start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGE(TAG, "BLE reset: %d", reason);
}

static void host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ═══════════════════════════════════════════════════════
// PUBLIC API
// ═══════════════════════════════════════════════════════

bool ble_servo_init(ble_servo_move_cb_t move_cb, 
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb) {
    ESP_LOGI(TAG, "Initializing BLE servo controller");
    
    s_move_cb = move_cb;
    s_stance_cb = stance_cb;
    s_connect_cb = connect_cb;
    
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return false;
    }
    
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    
    ble_svc_gap_device_name_set(BLE_SERVO_DEVICE_NAME);
    
    nimble_port_freertos_init(host_task);
    
    ESP_LOGI(TAG, "BLE ready - device: %s", BLE_SERVO_DEVICE_NAME);
    return true;
}

bool ble_servo_is_connected(void) {
    return s_connected;
}

bool ble_servo_send_response(const char* msg) {
    if (!s_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return false;
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!om) return false;
    
    int rc = ble_gatts_notify_custom(s_conn_handle, s_chr_handle, om);
    return rc == 0;
}

bool ble_servo_send_state(float fr, float fl, float br, float bl) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"pos\":[%.0f,%.0f,%.0f,%.0f]}", fr, fl, br, bl);
    return ble_servo_send_response(buf);
}

#else // BLE disabled

bool ble_servo_init(ble_servo_move_cb_t move_cb, 
                    ble_servo_stance_cb_t stance_cb,
                    ble_servo_connect_cb_t connect_cb) {
    ESP_LOGW(TAG, "BLE disabled in config");
    return false;
}

bool ble_servo_is_connected(void) { return false; }
bool ble_servo_send_response(const char* msg) { return false; }
bool ble_servo_send_state(float fr, float fl, float br, float bl) { return false; }

#endif
