#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_system.h"

void app_main(void){
    esp_rom_printf("[diag] step1: app_main entered\n");
    esp_rom_printf("[diag] step2: reset_reason=%d\n", (int)esp_reset_reason());
    for (int i = 0; i < 10; i++) {
        esp_rom_printf("[diag] step3.%d: alive at %dms\n", i, (int)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    esp_rom_printf("[diag] step4: idle loop\n");
    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}
