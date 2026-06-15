#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "app_config.h"

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
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t mpu6050_dev_handle = NULL;

void led_set(bool on);
void buzzer_set(bool on);
bool button_is_pressed(void);
esp_err_t board_i2c_init(void);
void i2c_scan(void);
esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data);
esp_err_t mpu6050_read_bytes(uint8_t start_reg, uint8_t *buffer, size_t len);
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);

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

// --- TASK 1: Đọc cảm biến (Mức ưu tiên cao nhất - Real-time) ---
void vSensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    TickType_t last_log_tick = 0;

    while (1) {
        // Sau này: Đọc MPU6050 qua I2C tại đây
        // Tần số lý tưởng cho AI phát hiện té ngã là 50Hz (20ms)
        int16_t ax = 0;
        int16_t ay = 0;
        int16_t az = 0;
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;
        esp_err_t err = mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_log_tick) >= pdMS_TO_TICKS(1000)) {
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "MPU6050 raw: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d", ax, ay, az, gx, gy, gz);
            } else {
                ESP_LOGW(TAG, "Failed to read MPU6050 raw data: %s", esp_err_to_name(err));
            }
            last_log_tick = now;
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS));
    }
}

// --- TASK 2: Xử lý AI/TinyML (Mức ưu tiên trung bình cao) ---
void vInferenceTask(void *pvParameters) {
    ESP_LOGI(TAG, "AI Inference Task Started");
    while (1) {
        // Sau này: Nhận dữ liệu từ sensor_data_queue và chạy model Edge Impulse
        // AI thường xử lý theo cửa sổ thời gian (ví dụ 2 giây một lần)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- TASK 3: Kết nối mạng & Cảnh báo từ xa (WiFi/MQTT) ---
void vNetworkTask(void *pvParameters) {
    ESP_LOGI(TAG, "Network/Cloud Task Started");
    while (1) {
        // Sau này: Gửi dữ liệu lên Server/App khi có tín hiệu từ alert_queue
        // Task này cần stack lớn hơn vì chạy WiFi stack
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- TASK 4: Cảnh báo tại chỗ & Giám sát hệ thống (Buzzer/Pin) ---
void vSystemMonitorTask(void *pvParameters) {
    ESP_LOGI(TAG, "System Monitor Task Started");
    led_set(false);
    buzzer_set(false);

    bool last_button_pressed = button_is_pressed();

    while (1) {
        // Sau này: Đọc ADC kiểm tra pin (TP4056), chớp LED trạng thái
        // Kích hoạt loa (Buzzer) nếu phát hiện té ngã
        bool button_pressed = button_is_pressed();
        if (button_pressed != last_button_pressed) {
            ESP_LOGI(TAG, "Button: %s", button_pressed ? "PRESSED" : "RELEASED");
            last_button_pressed = button_pressed;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- CHƯƠNG TRÌNH CHÍNH ---
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing Fall Detection System...");

    esp_err_t err = board_gpio_init();
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
    sensor_data_queue = xQueueCreate(10, sizeof(float) * 6); // Chứa 6 trục dữ liệu
    alert_queue = xQueueCreate(5, sizeof(bool));            // Chứa trạng thái Té ngã (True/False)

    // 2. Tạo các Task
    // Task đọc cảm biến (Priority: 5) - Cần chạy cực kỳ đúng giờ
    if (sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor_data_queue");
        return;
    }
    if (alert_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create alert_queue");
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
