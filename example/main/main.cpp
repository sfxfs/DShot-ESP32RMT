#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <DShotRMT.h>

static const char *TAG = "example";

DShotRMT motor1(GPIO_NUM_4, DSHOT600);

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Launching APP");

    // Arm Motors
    motor1.begin();

    int taskCounter = 0;

    while (true)
    {
        motor1.sendThrottleValue(100);

        taskCounter++;

        if (taskCounter > 10)
        {
            motor1.~DShotRMT();
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
