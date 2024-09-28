#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <DShotRMT.h>

static const char *TAG = "example";

DShotRMT motor(GPIO_NUM_4, DSHOT600_BIDIRECTIONAL);

static void rampThrottle(int start, int stop, int step)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        motor.sendThrottle(i);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    motor.sendThrottle(stop);
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing DShot RMT...");
    motor.begin();

    ESP_LOGI(TAG, "Ramping throttle...");
    int taskCounter = 0;
    int rampMax = DSHOT_THROTTLE_RANGE * 0.2;
    while (true)
    {
        rampThrottle(DSHOT_THROTTLE_MIN, rampMax, 1);
        rampThrottle(rampMax, DSHOT_THROTTLE_MIN, -1);

        taskCounter++;

        if (taskCounter >= 6)
        {
            break;
        }
    }
    motor.sendThrottle(0);

    ESP_LOGW(TAG, "Exiting");
}
