#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// UUID diconvert ke format Little-Endian (dibalik per-byte dari belakang ke depan)
// Service: 12345678-1234-1234-1234-1234567890ab
static const ble_uuid128_t shm_svc_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

// Char: abcd1234-5678-90ab-cdef-1234567890ab
static const ble_uuid128_t shm_chr_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab
);

typedef struct __attribute__((packed)) {
    float ax;
    float ay;
    float az;
} SHMData;

// Variabel Global
static uint8_t own_addr_type;
static uint16_t target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t notify_char_val_handle;
static SHMData current_shm_data; // Tempat menyimpan data terbaru

static void ble_app_advertise(void);

// 4. Callback GATT: Dipanggil otomatis oleh NimBLE saat Gateway membaca data atau di-notif
static int shm_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle, 
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle == notify_char_val_handle) {
        // Salin struct 'current_shm_data' ke buffer memory (mbuf) yang akan dikirim via Bluetooth
        int rc = os_mbuf_append(ctxt->om, &current_shm_data, sizeof(current_shm_data));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

// 3. Tabel Konfigurasi Service dan Karakteristik GATT
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &shm_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &shm_chr_uuid.u,
                .access_cb = shm_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &notify_char_val_handle,
            },
            { 0 } // Penutup array karakteristik
        }
    },
    { 0 } // Penutup array service
};

// 2. Fungsi menangani event Koneksi/Diskoneksi
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                printf("Gateway Connected!\n");
                target_conn_handle = event->connect.conn_handle;
            } else {
                ble_app_advertise(); // Jika gagal, mulai advertising lagi
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("Gateway Disconnected!\n");
            target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise(); // Mulai advertising lagi saat putus
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            printf("Gateway Subscribed to Notifikasi!\n");
            break;
    }
    return 0;
}

// 1. Fungsi memulai Advertising
static void ble_app_advertise(void) {
    // --- PAKET 1: ADVERTISING UTAMA (Maksimal 31 Byte) ---
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);

    // Set Flags standar
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // Set Service UUID di sini (Sangat penting agar Gateway bisa mendeteksi)
    fields.uuids128 = (ble_uuid128_t*)&shm_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    // Terapkan Paket 1
    ble_gap_adv_set_fields(&fields);


    // --- PAKET 2: SCAN RESPONSE (Maksimal 31 Byte tambahan) ---
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof rsp_fields);
    
    // Set Nama Perangkat di sini
    rsp_fields.name = (uint8_t *)"SHM_Node_C3";
    rsp_fields.name_len = strlen("SHM_Node_C3");
    rsp_fields.name_is_complete = 1;

    // Terapkan Paket 2
    ble_gap_adv_rsp_set_fields(&rsp_fields);


    // --- MULAI MEMANCARKAN ---
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Bisa dikoneksikan
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // General Discoverable

    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    printf("SHM Node advertising...\n");
}

// Sinkronisasi NimBLE
static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_app_advertise();
}

// Task untuk Mock Data Sensor
void shm_data_task(void *pvParameter) {
    while(1) {
        // HANYA proses data jika ada Gateway yang terkoneksi
        if (target_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            
            current_shm_data.ax = (float)((int)esp_random() % 400 - 200) / 100.0f;
            current_shm_data.ay = (float)((int)esp_random() % 400 - 200) / 100.0f;
            current_shm_data.az = (float)((int)esp_random() % 120 + 900) / 100.0f;

            // Trigger Notify ke Gateway (Fungsi ini bertipe void)
            ble_gatts_chr_updated(notify_char_val_handle);
            
            printf("[NODE] Data terkirim -> AX:%.2f AY:%.2f AZ:%.2f\n", 
                   current_shm_data.ax, current_shm_data.ay, current_shm_data.az);
        }
        
        // Delay 1 detik (Gunakan pdMS_TO_TICKS agar akurat di FreeRTOS)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Background Task NimBLE
void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    // 1. Inisialisasi Flash (Wajib)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. Inisialisasi NimBLE
    nimble_port_init();

    // 3. Daftarkan Service & Karakteristik GATT
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // 4. Konfigurasi Device
    ble_svc_gap_device_name_set("SHM_Node_C3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // 5. Jalankan Task
    nimble_port_freertos_init(ble_host_task);
    
    // 6. Buat task untuk Loop Sensor Data
    xTaskCreate(shm_data_task, "shm_task", 4096, NULL, 5, NULL);
}