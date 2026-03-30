#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --- TASK 1: Đọc cảm biến liên tục ---
void task_read_sensor(void *pvParameters) {
    while (1) {
        printf("Gia lap: Dang doc du lieu cam bien...\n");
        // Tạm dừng 100ms (10Hz) bằng hàm delay chuẩn của FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// --- TASK 2: Phân tích dữ liệu té ngã ---
void task_fall_detection(void *pvParameters) {
    while (1) {
        printf("Gia lap: Dang xu ly thuat toan te nga...\n");
        // Tạm dừng 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- CHƯƠNG TRÌNH CHÍNH ---
void app_main(void)
{
    printf("--- KHOI DONG HE THONG CANH BAO TE NGA ---\n");

    // Tạo Task 1: Ưu tiên cao hơn (mức 5) để đọc cảm biến không bị trễ
    xTaskCreate(task_read_sensor, "Read_Sensor", 2048, NULL, 5, NULL);

    // Tạo Task 2: Ưu tiên thấp hơn (mức 4) để xử lý logic
    xTaskCreate(task_fall_detection, "Fall_Detect", 4096, NULL, 4, NULL);
}