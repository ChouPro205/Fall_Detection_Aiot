#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "FALL_DETECTION_SYSTEM";

// --- KHAI BÁO HÀNG ĐỢI (QUEUES) ---
// Dùng để truyền dữ liệu thô từ cảm biến sang Task xử lý AI
QueueHandle_t sensor_data_queue;
// Dùng để truyền kết quả từ Task AI sang Task phát cảnh báo
QueueHandle_t alert_queue;

// --- TASK 1: Đọc cảm biến (Mức ưu tiên cao nhất - Real-time) ---
void vSensorTask(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor Task Started");
    while (1) {
        // Sau này: Đọc MPU6050 qua I2C tại đây
        // Tần số lý tưởng cho AI phát hiện té ngã là 50Hz (20ms)
        vTaskDelay(pdMS_TO_TICKS(20)); 
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
    while (1) {
        // Sau này: Đọc ADC kiểm tra pin (TP4056), chớp LED trạng thái
        // Kích hoạt loa (Buzzer) nếu phát hiện té ngã
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// --- CHƯƠNG TRÌNH CHÍNH ---
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing Fall Detection System...");

    // 1. Khởi tạo các Queue (Truyền tin giữa các Task)
    sensor_data_queue = xQueueCreate(10, sizeof(float) * 6); // Chứa 6 trục dữ liệu
    alert_queue = xQueueCreate(5, sizeof(bool));            // Chứa trạng thái Té ngã (True/False)

    // 2. Tạo các Task
    // Task đọc cảm biến (Priority: 5) - Cần chạy cực kỳ đúng giờ
    xTaskCreate(vSensorTask, "Sensor_Read", 4096, NULL, 5, NULL);

    // Task chạy AI (Priority: 4) - Xử lý tính toán nặng
    xTaskCreate(vInferenceTask, "AI_Inference", 8192, NULL, 4, NULL);

    // Task mạng (Priority: 3) - Kết nối WiFi/MQTT
    xTaskCreate(vNetworkTask, "Network_Alert", 8192, NULL, 3, NULL);

    // Task hệ thống (Priority: 2) - Quản lý pin, LED, còi
    xTaskCreate(vSystemMonitorTask, "Sys_Monitor", 2048, NULL, 2, NULL);

    ESP_LOGI(TAG, "All Tasks Created Successfully.");
}