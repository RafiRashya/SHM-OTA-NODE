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
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c.h" 

// --- PENYESUAIAN SISTEM 1: VERSI FIRMWARE ---
#define FIRMWARE_VERSION "v1.0.0"

// ================= KONFIGURASI I2C & ADXL345 =================
#define I2C_MASTER_SCL_IO           3       // Pin SCL
#define I2C_MASTER_SDA_IO           2       // Pin SDA
#define I2C_MASTER_NUM              0       // Port I2C (0)
#define I2C_MASTER_FREQ_HZ          400000  // Kecepatan I2C (400 kHz)

#define ADXL345_ADDR                0x53    // Alamat I2C ADXL345 (jika SDO ke GND)
#define ADXL345_REG_POWER_CTL       0x2D    // Register untuk menyalakan sensor
#define ADXL345_REG_DATA_FORMAT     0x31    // Register format data (resolusi)
#define ADXL345_REG_DATAX0          0x32    // Register awal data sumbu X, Y, Z

// UUID diconvert ke format Little-Endian
static const ble_uuid128_t shm_svc_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
);

static const ble_uuid128_t shm_chr_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab
);

// === UUID UNTUK OTA SERVICE ===
static const ble_uuid128_t ota_svc_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12
);

static const ble_uuid128_t ota_chr_ctrl_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 
    0x00, 0x00, 0x01, 0x00, 0x34, 0x12, 0xcd, 0xab
);

static const ble_uuid128_t ota_chr_data_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 
    0x00, 0x00, 0x02, 0x00, 0x34, 0x12, 0xcd, 0xab
);

// --- PENYESUAIAN SISTEM 2: UUID VERSI ---
static const ble_uuid128_t ota_chr_ver_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 
    0x00, 0x00, 0x03, 0x00, 0x34, 0x12, 0xcd, 0xab
);

static int ota_gatt_ver_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = os_mbuf_append(ctxt->om, FIRMWARE_VERSION, strlen(FIRMWARE_VERSION));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}
// ----------------------------------------

// === VARIABEL TRACKING OTA ===
static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t *update_partition = NULL;
static bool ota_is_running = false;
static size_t ota_total_bytes = 0;

static int ota_gatt_ctrl_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ota_gatt_data_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

typedef struct __attribute__((packed)) {
    float ax;
    float ay;
    float az;
    float vbatt;
} SHMData;

// Variabel Global
static uint8_t own_addr_type;
static uint16_t target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t notify_char_val_handle;
static SHMData current_shm_data; 

// === HANDLE ADC ===
static adc_oneshot_unit_handle_t adc1_handle;

static void ble_app_advertise(void);

// ================= FUNGSI INISIALISASI I2C DAN ADXL345 =================
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void adxl345_init(void) {
    uint8_t format_cmd[2] = {ADXL345_REG_DATA_FORMAT, 0x0B};
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345_ADDR, format_cmd, sizeof(format_cmd), 1000 / portTICK_PERIOD_MS);

    uint8_t power_cmd[2] = {ADXL345_REG_POWER_CTL, 0x08};
    i2c_master_write_to_device(I2C_MASTER_NUM, ADXL345_ADDR, power_cmd, sizeof(power_cmd), 1000 / portTICK_PERIOD_MS);
    
    printf("[SENSOR] ADXL345 Berhasil Diinisialisasi!\n");
}
// =======================================================================


static int shm_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle, 
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle == notify_char_val_handle) {
        int rc = os_mbuf_append(ctxt->om, &current_shm_data, sizeof(current_shm_data));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int ota_gatt_ctrl_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t cmd;
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        
        if (len != 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        os_mbuf_copydata(ctxt->om, 0, 1, &cmd);

        if (cmd == 0x01) { 
            printf("\n[OTA] Menerima perintah START (0x01)\n");
            update_partition = esp_ota_get_next_update_partition(NULL);
            if (update_partition == NULL) {
                printf("[OTA] Error: Partisi OTA tidak ditemukan!\n");
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            printf("[OTA] Menulis ke partisi: %s\n", update_partition->label);
            esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
            if (err != ESP_OK) {
                printf("[OTA] Error esp_ota_begin: %s\n", esp_err_to_name(err));
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            ota_is_running = true;
            ota_total_bytes = 0;
            printf("[OTA] OTA Dimulai! Menyiapkan pipa data...\n");
        }
        else if (cmd == 0x02) { 
            if (!ota_is_running) return BLE_ATT_ERR_UNLIKELY;
            printf("\n[OTA] Menerima perintah END (0x02). Total data: %d bytes\n", ota_total_bytes);
            
            esp_err_t err = esp_ota_end(ota_handle);
            if (err != ESP_OK) {
                printf("[OTA] Error esp_ota_end: %s\n", esp_err_to_name(err));
                ota_is_running = false;
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            err = esp_ota_set_boot_partition(update_partition);
            if (err != ESP_OK) {
                printf("[OTA] Error mengatur partisi boot: %s\n", esp_err_to_name(err));
                ota_is_running = false;
                return BLE_ATT_ERR_UNLIKELY;
            }
            
            printf("[OTA] FIRMWARE BERHASIL DIPERBARUI! Restarting dalam 3 detik...\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_restart(); 
        }
    }
    return 0;
}

static int ota_gatt_data_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (!ota_is_running) return BLE_ATT_ERR_UNLIKELY; 

        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t data_buf[512]; 
        if (len > sizeof(data_buf)) return BLE_ATT_ERR_INSUFFICIENT_RES;

        os_mbuf_copydata(ctxt->om, 0, len, data_buf);

        esp_err_t err = esp_ota_write(ota_handle, data_buf, len);
        if (err != ESP_OK) {
            printf("[OTA] Gagal menulis ke Flash: %s\n", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            ota_is_running = false;
            return BLE_ATT_ERR_UNLIKELY;
        }
        
        ota_total_bytes += len;
        
        // Cukup cetak setiap 10KB agar tidak memberatkan Node
        if (ota_total_bytes % 10240 < len) { 
            printf("[OTA] Menerima data: %d bytes...\n", ota_total_bytes);
        }
    }
    return 0;
}

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
            { 0 }
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ota_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &ota_chr_ctrl_uuid.u,
                .access_cb = ota_gatt_ctrl_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &ota_chr_data_uuid.u,
                .access_cb = ota_gatt_data_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP, 
            },
            // --- PENYESUAIAN SISTEM 3: MENAMBAHKAN KARAKTERISTIK VERSI ---
            {
                .uuid = &ota_chr_ver_uuid.u,
                .access_cb = ota_gatt_ver_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        }
    },
    { 0 } 
};

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                printf("Gateway Connected!\n");
                target_conn_handle = event->connect.conn_handle;
            } else {
                ble_app_advertise(); 
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            printf("Gateway Disconnected!\n");
            target_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_app_advertise(); 
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            printf("Gateway Subscribed to Notifikasi!\n");
            break;
    }
    return 0;
}

static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t*)&shm_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof rsp_fields);
    
    rsp_fields.name = (uint8_t *)"SHM_Node_C3";
    rsp_fields.name_len = strlen("SHM_Node_C3");
    rsp_fields.name_is_complete = 1;

    ble_gap_adv_rsp_set_fields(&rsp_fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 

    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    printf("SHM Node advertising...\n");
}

static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_app_advertise();
}

// ================= TASK SENSOR UTAMA =================
void shm_data_task(void *pvParameter) {
    uint8_t data_raw[6]; 

    while(1) {
        // HAPUS SYARAT !ota_is_running AGAR SENSOR TETAP AKTIF SAAT OTA
        if (target_conn_handle != BLE_HS_CONN_HANDLE_NONE && !ota_is_running) {
            
            int raw_val = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &raw_val));
            float voltage = (raw_val / 4095.0) * 3.3 * 1.83; 

            uint8_t reg_addr = ADXL345_REG_DATAX0;
            
            // 1. TANGKAP STATUS ERROR I2C
            esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, ADXL345_ADDR, &reg_addr, 1, data_raw, 6, 1000 / portTICK_PERIOD_MS);

            // 2. JIKA SUKSES, UPDATE DATA
            if (err == ESP_OK) {
                int16_t x_int = (data_raw[1] << 8) | data_raw[0];
                int16_t y_int = (data_raw[3] << 8) | data_raw[2];
                int16_t z_int = (data_raw[5] << 8) | data_raw[4];

                current_shm_data.ax = x_int * 0.0039f;
                current_shm_data.ay = y_int * 0.0039f;
                current_shm_data.az = z_int * 0.0039f;
            } else {
                // 3. JIKA GAGAL, CETAK ERROR KE SERIAL & NOL KAN DATA
                printf("[HW ERROR] Gagal membaca ADXL345! Cek Kabel (Status: %s)\n", esp_err_to_name(err));
                current_shm_data.ax = 0.0f;
                current_shm_data.ay = 0.0f;
                current_shm_data.az = 0.0f;
            }

            current_shm_data.vbatt = voltage; 

            ble_gatts_chr_updated(notify_char_val_handle);
            
            printf("[NODE] AX:%.2f AY:%.2f AZ:%.2f | Vbatt: %.2fV\n", 
                   current_shm_data.ax, current_shm_data.ay, current_shm_data.az, current_shm_data.vbatt);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// =====================================================

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, 
        .atten = ADC_ATTEN_DB_12,         
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));

    ESP_ERROR_CHECK(i2c_master_init());

    // === KODE I2C SCANNER ESP-IDF V5 ===
    printf("\n[RADAR] Memulai I2C Scanner...\n");
    int devices_found = 0;
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true); // Kirim 1 bit ACK
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("[RADAR] ==> SENSOR DITEMUKAN PADA ALAMAT: 0x%02X <==\n", i);
            devices_found++;
        }
    }
    if (devices_found == 0) {
        printf("[RADAR] KOSONG! Tidak ada satupun sensor yang terdeteksi.\n");
    }
    printf("[RADAR] Scan Selesai.\n\n");
    // ===================================
    adxl345_init();

    nimble_port_init();
    
    // --- PENYESUAIAN SISTEM 5: UNCOMMENT MTU 512 ---
    ble_att_set_preferred_mtu(512);

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    ble_svc_gap_device_name_set("SHM_Node_C3");
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_freertos_init(ble_host_task);
    
    xTaskCreate(shm_data_task, "shm_task", 4096, NULL, 5, NULL);
}