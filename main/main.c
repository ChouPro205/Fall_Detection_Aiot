#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "app_config.h"

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#include "wifi_credentials_example.h"
#endif

#if __has_include("telegram_credentials.h")
#include "telegram_credentials.h"
#else
#include "telegram_credentials_example.h"
#endif

static const char *TAG = "FALL_DETECTION_SYSTEM";
static const int I2C_SCAN_TIMEOUT_MS = 50;
static const int I2C_DEVICE_TIMEOUT_MS = 100;
static const uint32_t I2C_DEVICE_SPEED_HZ = 100000;
static const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_REG_WHO_AM_I = 0x75;
static const uint8_t MPU6050_WHO_AM_I_VALUE = 0x68;
static const float MPU6050_ACCEL_SCALE = 16384.0f;
static const float MPU6050_GYRO_SCALE = 131.0f;
static const float FREE_FALL_THRESHOLD_G = 0.5f;
static const float IMPACT_THRESHOLD_G = 2.5f;
static const float STABLE_ACCEL_MIN_G = 0.8f;
static const float STABLE_ACCEL_MAX_G = 1.2f;
static const float STABLE_GYRO_THRESHOLD_DPS = 30.0f;
static const int64_t POSTURE_CHECK_TIME_MS = 3000;
static const int64_t ALERT_COOLDOWN_MS = 10000;
static const int64_t FREE_FALL_TIMEOUT_MS = 1000;
static const int64_t LOCAL_WARNING_TIMEOUT_MS = 10000;
static const int64_t ALERT_LED_TOGGLE_PERIOD_MS = 250;
static const int64_t ALERT_BUZZER_PERIOD_MS = 500;
static const int64_t ALERT_BUZZER_ON_TIME_MS = 120;
static const int64_t BUTTON_DEBOUNCE_MS = 50;
static const int64_t SYSTEM_MONITOR_PERIOD_MS = 50;
static const int WIFI_MAXIMUM_RETRY = 5;
static const int64_t NETWORK_ALERT_COOLDOWN_MS = 30000;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_dev_handle = NULL;
static EventGroupHandle_t wifi_event_group = NULL;
static int wifi_retry_count = 0;

typedef struct {
    int64_t timestamp_ms;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} sensor_sample_t;

typedef enum {
    FALL_STATE_NORMAL,
    FALL_STATE_FREE_FALL,
    FALL_STATE_IMPACT,
    FALL_STATE_POSTURE_CHECK,
    FALL_STATE_CONFIRMED,
} fall_state_t;

typedef enum {
    ALERT_STATE_IDLE,
    ALERT_STATE_LOCAL_WARNING,
    ALERT_STATE_CANCELLED,
    ALERT_STATE_CONFIRMED,
} alert_state_t;

void led_set(bool on);
void buzzer_set(bool on);
bool button_is_pressed(void);
esp_err_t app_nvs_init(void);
esp_err_t board_i2c_init(void);
void i2c_scan(void);
esp_err_t wifi_init_sta(void);
esp_err_t telegram_send_message(const char *message);
esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data);
esp_err_t mpu6050_read_bytes(uint8_t start_reg, uint8_t *buffer, size_t len);
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

static const char *fall_state_to_string(fall_state_t state)
{
    switch (state) {
    case FALL_STATE_NORMAL:
        return "NORMAL";
    case FALL_STATE_FREE_FALL:
        return "FREE_FALL";
    case FALL_STATE_IMPACT:
        return "IMPACT";
    case FALL_STATE_POSTURE_CHECK:
        return "POSTURE_CHECK";
    case FALL_STATE_CONFIRMED:
        return "CONFIRMED";
    default:
        return "UNKNOWN";
    }
}

static void fall_state_transition(fall_state_t *state, fall_state_t next_state, int64_t timestamp_ms)
{
    if (*state == next_state) {
        return;
    }

    ESP_LOGI(
        TAG,
        "Fall state: %s -> %s at t=%" PRId64 "ms",
        fall_state_to_string(*state),
        fall_state_to_string(next_state),
        timestamp_ms
    );
    *state = next_state;
}

esp_err_t app_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init failed, erasing NVS partition");
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_flash_init();
    }

    return err;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_event_group != NULL) {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }

        if (wifi_retry_count < WIFI_MAXIMUM_RETRY) {
            wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi disconnected, reconnecting (%d/%d)", wifi_retry_count, WIFI_MAXIMUM_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "WiFi disconnected, max retry reached");
            if (wifi_event_group != NULL) {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        if (wifi_event_group != NULL) {
            xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASSWORD);
    wifi_config.sta.threshold.authmode = strlen(WIFI_PASSWORD) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
        ESP_LOGW(TAG, "Using placeholder WiFi credentials. Create main/wifi_credentials.h before flashing real hardware.");
    }

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "WiFi station initialized");
    return ESP_OK;
}

esp_err_t telegram_send_message(const char *message)
{
    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Telegram message sending...");

    if (strcmp(TELEGRAM_BOT_TOKEN, "YOUR_TELEGRAM_BOT_TOKEN") == 0 ||
        strcmp(TELEGRAM_CHAT_ID, "YOUR_TELEGRAM_CHAT_ID") == 0) {
        ESP_LOGW(TAG, "Telegram credentials are placeholders. Create main/telegram_credentials.h before testing Telegram.");
        ESP_LOGE(TAG, "Telegram message failed");
        return ESP_ERR_INVALID_STATE;
    }

    char url[256] = {0};
    int written = snprintf(
        url,
        sizeof(url),
        "https://api.telegram.org/bot%s/sendMessage",
        TELEGRAM_BOT_TOKEN
    );
    if (written < 0 || written >= (int)sizeof(url)) {
        ESP_LOGE(TAG, "Telegram URL buffer too small");
        ESP_LOGE(TAG, "Telegram message failed");
        return ESP_ERR_INVALID_SIZE;
    }

    char post_data[512] = {0};
    written = snprintf(
        post_data,
        sizeof(post_data),
        "{\"chat_id\":\"%s\",\"text\":\"%s\"}",
        TELEGRAM_CHAT_ID,
        message
    );
    if (written < 0 || written >= (int)sizeof(post_data)) {
        ESP_LOGE(TAG, "Telegram POST buffer too small");
        ESP_LOGE(TAG, "Telegram message failed");
        return ESP_ERR_INVALID_SIZE;
    }

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Telegram HTTP client init failed");
        ESP_LOGE(TAG, "Telegram message failed");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code >= 200 && status_code < 300) {
            ESP_LOGI(TAG, "Telegram message sent OK");
        } else {
            ESP_LOGE(TAG, "Telegram message failed, HTTP status=%d", status_code);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Telegram message failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

esp_err_t board_gpio_init(void)
{
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << LED_GPIO) | (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&output_config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&button_config);
    if (err != ESP_OK) {
        return err;
    }

    led_set(false);
    buzzer_set(false);

    return ESP_OK;
}

void led_set(bool on)
{
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

void buzzer_set(bool on)
{
    gpio_set_level(BUZZER_GPIO, on ? 1 : 0);
}

bool button_is_pressed(void)
{
    return gpio_get_level(BUTTON_GPIO) == 0;
}

esp_err_t board_i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)MPU6050_SDA_GPIO,
        .scl_io_num = (gpio_num_t)MPU6050_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    return i2c_new_master_bus(&bus_config, &i2c_bus_handle);
}

void i2c_scan(void)
{
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus is not initialized");
        return;
    }

    bool device_found = false;
    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t address = 0x03; address <= 0x77; address++) {
        esp_err_t err = i2c_master_probe(i2c_bus_handle, address, I2C_SCAN_TIMEOUT_MS);
        if (err == ESP_OK) {
            device_found = true;
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", address);

            if (address == MPU6050_I2C_ADDR) {
                ESP_LOGI(TAG, "MPU6050 found");
            }
        }
    }

    if (!device_found) {
        ESP_LOGW(TAG, "No I2C devices found. Check SDA/SCL/3V3/GND.");
    }
}

static esp_err_t mpu6050_get_device_handle(void)
{
    if (mpu6050_dev_handle != NULL) {
        return ESP_OK;
    }

    if (i2c_bus_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_DEVICE_SPEED_HZ,
    };

    return i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &mpu6050_dev_handle);
}

static int16_t mpu6050_parse_int16(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8) | low_byte);
}

esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    esp_err_t err = mpu6050_get_device_handle();
    if (err != ESP_OK) {
        return err;
    }

    uint8_t write_buffer[2] = {reg, data};
    return i2c_master_transmit(mpu6050_dev_handle, write_buffer, sizeof(write_buffer), I2C_DEVICE_TIMEOUT_MS);
}

esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return mpu6050_read_bytes(reg, data, 1);
}

esp_err_t mpu6050_read_bytes(uint8_t start_reg, uint8_t *buffer, size_t len)
{
    if (buffer == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mpu6050_get_device_handle();
    if (err != ESP_OK) {
        return err;
    }

    return i2c_master_transmit_receive(
        mpu6050_dev_handle,
        &start_reg,
        sizeof(start_reg),
        buffer,
        len,
        I2C_DEVICE_TIMEOUT_MS
    );
}

esp_err_t mpu6050_init(void)
{
    uint8_t who_am_i = 0;
    esp_err_t err = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X", who_am_i);
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "Unexpected MPU6050 WHO_AM_I value");
        return ESP_FAIL;
    }

    err = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    err = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050 accelerometer: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050 gyroscope: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU6050 initialized");
    return ESP_OK;
}

esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    if (ax == NULL || ay == NULL || az == NULL || gx == NULL || gy == NULL || gz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[14] = {0};
    esp_err_t err = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    *ax = mpu6050_parse_int16(data[0], data[1]);
    *ay = mpu6050_parse_int16(data[2], data[3]);
    *az = mpu6050_parse_int16(data[4], data[5]);
    *gx = mpu6050_parse_int16(data[8], data[9]);
    *gy = mpu6050_parse_int16(data[10], data[11]);
    *gz = mpu6050_parse_int16(data[12], data[13]);

    return ESP_OK;
}

// --- KHAI BÁO HÀNG ĐỢI (QUEUES) ---
// Dùng để truyền dữ liệu thô từ cảm biến sang Task xử lý AI
QueueHandle_t sensor_data_queue;
// Dùng để truyền kết quả từ Task AI sang Task phát cảnh báo
QueueHandle_t alert_queue;
QueueHandle_t network_alert_queue;

// --- TASK 1: Đọc cảm biến (Mức ưu tiên cao nhất - Real-time) ---
void vSensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    TickType_t last_wake_tick = xTaskGetTickCount();
    TickType_t last_queue_warning_tick = 0;
    const TickType_t sample_period_ticks = pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS);

    while (1) {
        // Sau này: Đọc MPU6050 qua I2C tại đây
        // Tần số lý tưởng cho AI phát hiện té ngã là 50Hz (20ms)
        int16_t ax_raw = 0;
        int16_t ay_raw = 0;
        int16_t az_raw = 0;
        int16_t gx_raw = 0;
        int16_t gy_raw = 0;
        int16_t gz_raw = 0;
        esp_err_t err = mpu6050_read_raw(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        if (err == ESP_OK) {
            sensor_sample_t sample = {
                .timestamp_ms = esp_timer_get_time() / 1000,
                .ax_g = (float)ax_raw / MPU6050_ACCEL_SCALE,
                .ay_g = (float)ay_raw / MPU6050_ACCEL_SCALE,
                .az_g = (float)az_raw / MPU6050_ACCEL_SCALE,
                .gx_dps = (float)gx_raw / MPU6050_GYRO_SCALE,
                .gy_dps = (float)gy_raw / MPU6050_GYRO_SCALE,
                .gz_dps = (float)gz_raw / MPU6050_GYRO_SCALE,
            };

            if (xQueueSend(sensor_data_queue, &sample, 0) != pdPASS) {
                TickType_t now = xTaskGetTickCount();
                if ((now - last_queue_warning_tick) >= pdMS_TO_TICKS(1000)) {
                    ESP_LOGW(TAG, "sensor_data_queue full, dropping sample");
                    last_queue_warning_tick = now;
                }
            }
        } else {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_queue_warning_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGW(TAG, "Failed to read MPU6050 raw data: %s", esp_err_to_name(err));
                last_queue_warning_tick = now;
            }
        }

        vTaskDelayUntil(&last_wake_tick, sample_period_ticks);
    }
}

// --- TASK 2: Xử lý AI/TinyML (Mức ưu tiên trung bình cao) ---
void vInferenceTask(void *pvParameters) {
    ESP_LOGI(TAG, "AI Inference Task Started");
    TickType_t last_log_tick = 0;
#if ENABLE_SENSOR_CSV_LOG
    bool csv_header_printed = false;
#endif
    fall_state_t fall_state = FALL_STATE_NORMAL;
    int64_t free_fall_start_ms = 0;
    int64_t impact_time_ms = 0;
    int64_t stable_start_ms = 0;
    int64_t last_alert_ms = -ALERT_COOLDOWN_MS;

    while (1) {
        // Sau này: Nhận dữ liệu từ sensor_data_queue và chạy model Edge Impulse
        // AI thường xử lý theo cửa sổ thời gian (ví dụ 2 giây một lần)
        sensor_sample_t sample;
        if (xQueueReceive(sensor_data_queue, &sample, portMAX_DELAY) == pdPASS) {
            TickType_t now = xTaskGetTickCount();
            float accel_magnitude = sqrtf(
                (sample.ax_g * sample.ax_g) +
                (sample.ay_g * sample.ay_g) +
                (sample.az_g * sample.az_g)
            );
            float gyro_magnitude = sqrtf(
                (sample.gx_dps * sample.gx_dps) +
                (sample.gy_dps * sample.gy_dps) +
                (sample.gz_dps * sample.gz_dps)
            );

#if ENABLE_SENSOR_CSV_LOG
            if (!csv_header_printed) {
                puts("timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,A,G,label");
                csv_header_printed = true;
            }

            printf(
                "%" PRId64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
                sample.timestamp_ms,
                sample.ax_g,
                sample.ay_g,
                sample.az_g,
                sample.gx_dps,
                sample.gy_dps,
                sample.gz_dps,
                accel_magnitude,
                gyro_magnitude,
                SENSOR_CSV_LABEL
            );
#endif

            if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
                ESP_LOGI(
                    TAG,
                    "Sensor sample: t=%" PRId64 "ms ax=%.3fg ay=%.3fg az=%.3fg gx=%.2fdps gy=%.2fdps gz=%.2fdps A=%.3fg G=%.2fdps",
                    sample.timestamp_ms,
                    sample.ax_g,
                    sample.ay_g,
                    sample.az_g,
                    sample.gx_dps,
                    sample.gy_dps,
                    sample.gz_dps,
                    accel_magnitude,
                    gyro_magnitude
                );
                last_log_tick = now;
            }

            switch (fall_state) {
            case FALL_STATE_NORMAL:
                stable_start_ms = 0;
                if (accel_magnitude < FREE_FALL_THRESHOLD_G) {
                    free_fall_start_ms = sample.timestamp_ms;
                    fall_state_transition(&fall_state, FALL_STATE_FREE_FALL, sample.timestamp_ms);
                } else if (accel_magnitude > IMPACT_THRESHOLD_G) {
                    impact_time_ms = sample.timestamp_ms;
                    fall_state_transition(&fall_state, FALL_STATE_IMPACT, sample.timestamp_ms);
                }
                break;

            case FALL_STATE_FREE_FALL:
                if (accel_magnitude > IMPACT_THRESHOLD_G) {
                    impact_time_ms = sample.timestamp_ms;
                    fall_state_transition(&fall_state, FALL_STATE_IMPACT, sample.timestamp_ms);
                } else if ((sample.timestamp_ms - free_fall_start_ms) > FREE_FALL_TIMEOUT_MS) {
                    free_fall_start_ms = 0;
                    fall_state_transition(&fall_state, FALL_STATE_NORMAL, sample.timestamp_ms);
                }
                break;

            case FALL_STATE_IMPACT:
                stable_start_ms = 0;
                fall_state_transition(&fall_state, FALL_STATE_POSTURE_CHECK, sample.timestamp_ms);
                break;

            case FALL_STATE_POSTURE_CHECK: {
                bool posture_stable =
                    (accel_magnitude >= STABLE_ACCEL_MIN_G) &&
                    (accel_magnitude <= STABLE_ACCEL_MAX_G) &&
                    (gyro_magnitude < STABLE_GYRO_THRESHOLD_DPS);

                if (posture_stable) {
                    if (stable_start_ms == 0) {
                        stable_start_ms = sample.timestamp_ms;
                    }

                    if ((sample.timestamp_ms - stable_start_ms) >= POSTURE_CHECK_TIME_MS) {
                        fall_state_transition(&fall_state, FALL_STATE_CONFIRMED, sample.timestamp_ms);
                    }
                } else {
                    stable_start_ms = 0;
                    if ((sample.timestamp_ms - impact_time_ms) >= POSTURE_CHECK_TIME_MS) {
                        fall_state_transition(&fall_state, FALL_STATE_NORMAL, sample.timestamp_ms);
                    }
                }
                break;
            }

            case FALL_STATE_CONFIRMED:
                if ((sample.timestamp_ms - last_alert_ms) >= ALERT_COOLDOWN_MS) {
                    ESP_LOGW(TAG, "FALL DETECTED at t=%" PRId64 "ms", sample.timestamp_ms);
                    last_alert_ms = sample.timestamp_ms;

                    if (alert_queue != NULL) {
                        bool alert_event = true;
                        if (xQueueSend(alert_queue, &alert_event, 0) != pdPASS) {
                            ESP_LOGW(TAG, "alert_queue full, dropping alert event");
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "Fall detected during cooldown, alert suppressed");
                }

                free_fall_start_ms = 0;
                impact_time_ms = 0;
                stable_start_ms = 0;
                fall_state_transition(&fall_state, FALL_STATE_NORMAL, sample.timestamp_ms);
                break;

            default:
                free_fall_start_ms = 0;
                impact_time_ms = 0;
                stable_start_ms = 0;
                fall_state_transition(&fall_state, FALL_STATE_NORMAL, sample.timestamp_ms);
                break;
            }
        }
    }
}

// --- TASK 3: Kết nối mạng & Cảnh báo từ xa (WiFi/MQTT) ---
void vNetworkTask(void *pvParameters) {
    ESP_LOGI(TAG, "Network/Cloud Task Started");
    ESP_LOGI(TAG, "NetworkTask ready");
    bool ready_logged = false;
    bool telegram_test_sent = false;
    int64_t last_network_alert_ms = -NETWORK_ALERT_COOLDOWN_MS;
    const char *fall_alert_message =
        "CANH BAO TE NGA!\n"
        "ESP32 da xac nhan su kien te nga.\n"
        "Nguoi dung khong bam nut huy.\n"
        "Vui long kiem tra ngay.";

    while (1) {
        // Sau này: Gửi dữ liệu lên Server/App khi có tín hiệu từ alert_queue
        // Task này cần stack lớn hơn vì chạy WiFi stack
        if (wifi_event_group == NULL) {
            ESP_LOGW(TAG, "WiFi event group is not initialized");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(1000)
        );

        if ((bits & WIFI_CONNECTED_BIT) != 0) {
            if (!ready_logged) {
                ESP_LOGI(TAG, "WiFi connected, ready for Telegram step");
                ready_logged = true;
            }

            if (!telegram_test_sent) {
                esp_err_t err = telegram_send_message("ESP32 fall detection system online. Telegram test OK.");
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Telegram test message attempt finished with error: %s", esp_err_to_name(err));
                }
                telegram_test_sent = true;
            }

            bool confirmed_fall_event = false;
            if (network_alert_queue != NULL &&
                xQueueReceive(network_alert_queue, &confirmed_fall_event, pdMS_TO_TICKS(1000)) == pdPASS &&
                confirmed_fall_event) {
                int64_t now_ms = esp_timer_get_time() / 1000;
                if ((now_ms - last_network_alert_ms) >= NETWORK_ALERT_COOLDOWN_MS) {
                    esp_err_t err = telegram_send_message(fall_alert_message);
                    if (err == ESP_OK) {
                        last_network_alert_ms = now_ms;
                    } else {
                        ESP_LOGW(TAG, "Fall Telegram alert failed: %s", esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGW(TAG, "Fall Telegram alert suppressed by cooldown");
                }
            }
        } else if ((bits & WIFI_FAIL_BIT) != 0) {
            ESP_LOGW(TAG, "WiFi connection failed, retrying periodically");
            xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
            wifi_retry_count = 0;
            ready_logged = false;
            esp_wifi_connect();
        } else {
            ready_logged = false;
        }
    }
}

// --- TASK 4: Cảnh báo tại chỗ & Giám sát hệ thống (Buzzer/Pin) ---
void vSystemMonitorTask(void *pvParameters) {
    ESP_LOGI(TAG, "System Monitor Task Started");
    led_set(false);
    buzzer_set(false);

    alert_state_t alert_state = ALERT_STATE_IDLE;
    int64_t alert_start_ms = 0;
    int64_t last_led_toggle_ms = 0;
    bool led_on = false;

    bool raw_button_pressed = button_is_pressed();
    bool last_raw_button_pressed = raw_button_pressed;
    bool debounced_button_pressed = raw_button_pressed;
    bool last_logged_button_pressed = debounced_button_pressed;
    int64_t button_last_change_ms = esp_timer_get_time() / 1000;

    while (1) {
        // Sau này: Đọc ADC kiểm tra pin (TP4056), chớp LED trạng thái
        // Kích hoạt loa (Buzzer) nếu phát hiện té ngã
        int64_t now_ms = esp_timer_get_time() / 1000;

        raw_button_pressed = button_is_pressed();
        if (raw_button_pressed != last_raw_button_pressed) {
            last_raw_button_pressed = raw_button_pressed;
            button_last_change_ms = now_ms;
        }

        if ((raw_button_pressed != debounced_button_pressed) &&
            ((now_ms - button_last_change_ms) >= BUTTON_DEBOUNCE_MS)) {
            debounced_button_pressed = raw_button_pressed;
        }

        if (debounced_button_pressed != last_logged_button_pressed) {
            ESP_LOGI(TAG, "Button: %s", debounced_button_pressed ? "PRESSED" : "RELEASED");
            last_logged_button_pressed = debounced_button_pressed;
        }

        bool alert_event = false;
        while (alert_queue != NULL && xQueueReceive(alert_queue, &alert_event, 0) == pdPASS) {
            if (alert_event) {
                ESP_LOGI(TAG, "Alert event received");
                if (alert_state == ALERT_STATE_IDLE) {
                    alert_state = ALERT_STATE_LOCAL_WARNING;
                    alert_start_ms = now_ms;
                    last_led_toggle_ms = now_ms;
                    led_on = true;
                    led_set(led_on);
                    buzzer_set(true);
                    ESP_LOGW(TAG, "Local fall warning started");
                }
            }
        }

        switch (alert_state) {
        case ALERT_STATE_IDLE:
            led_on = false;
            led_set(false);
            buzzer_set(false);
            break;

        case ALERT_STATE_LOCAL_WARNING:
            if ((now_ms - last_led_toggle_ms) >= ALERT_LED_TOGGLE_PERIOD_MS) {
                led_on = !led_on;
                led_set(led_on);
                last_led_toggle_ms = now_ms;
            }

            buzzer_set(((now_ms - alert_start_ms) % ALERT_BUZZER_PERIOD_MS) < ALERT_BUZZER_ON_TIME_MS);

            if (debounced_button_pressed) {
                alert_state = ALERT_STATE_CANCELLED;
            } else if ((now_ms - alert_start_ms) >= LOCAL_WARNING_TIMEOUT_MS) {
                alert_state = ALERT_STATE_CONFIRMED;
            }
            break;

        case ALERT_STATE_CANCELLED:
            led_on = false;
            led_set(false);
            buzzer_set(false);
            ESP_LOGI(TAG, "Fall alert cancelled by button");
            alert_state = ALERT_STATE_IDLE;
            break;

        case ALERT_STATE_CONFIRMED:
            led_on = false;
            led_set(false);
            buzzer_set(false);
            ESP_LOGW(TAG, "Fall alert confirmed");
            if (network_alert_queue != NULL) {
                bool confirmed_fall_event = true;
                if (xQueueSend(network_alert_queue, &confirmed_fall_event, 0) != pdPASS) {
                    ESP_LOGW(TAG, "network_alert_queue full, dropping confirmed fall event");
                }
            }
            alert_state = ALERT_STATE_IDLE;
            break;

        default:
            led_on = false;
            led_set(false);
            buzzer_set(false);
            alert_state = ALERT_STATE_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_MONITOR_PERIOD_MS));
    }
}

// --- CHƯƠNG TRÌNH CHÍNH ---
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing Fall Detection System...");

    esp_err_t err = app_nvs_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        return;
    }

    err = board_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize board GPIO: %s", esp_err_to_name(err));
        return;
    }

    err = board_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C master: %s", esp_err_to_name(err));
        return;
    }
    i2c_scan();

    err = mpu6050_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(err));
        return;
    }

    // 1. Khởi tạo các Queue (Truyền tin giữa các Task)
    err = wifi_init_sta();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi station: %s", esp_err_to_name(err));
        return;
    }

    sensor_data_queue = xQueueCreate(10, sizeof(sensor_sample_t)); // Normalized sensor samples
    alert_queue = xQueueCreate(5, sizeof(bool));            // Chứa trạng thái Té ngã (True/False)

    // 2. Tạo các Task
    // Task đọc cảm biến (Priority: 5) - Cần chạy cực kỳ đúng giờ
    network_alert_queue = xQueueCreate(3, sizeof(bool));

    if (sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor_data_queue");
        return;
    }
    if (alert_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create alert_queue");
        return;
    }
    if (network_alert_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create network_alert_queue");
        return;
    }
    if (xTaskCreate(vSensorTask, "Sensor_Read", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Sensor_Read task");
        return;
    }

    // Task chạy AI (Priority: 4) - Xử lý tính toán nặng
    if (xTaskCreate(vInferenceTask, "AI_Inference", 8192, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create AI_Inference task");
        return;
    }

    // Task mạng (Priority: 3) - Kết nối WiFi/MQTT
    if (xTaskCreate(vNetworkTask, "Network_Alert", 8192, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Network_Alert task");
        return;
    }

    // Task hệ thống (Priority: 2) - Quản lý pin, LED, còi
    if (xTaskCreate(vSystemMonitorTask, "Sys_Monitor", 2048, NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Sys_Monitor task");
        return;
    }

    ESP_LOGI(TAG, "All Tasks Created Successfully.");
}
