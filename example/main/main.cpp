#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <DShotRMT.h>

static const char *TAG = "example";

typedef struct log_entry_t
{
    uint16_t throttle;
    uint32_t erpm;
    uint32_t tick;
};

DShotRMT motor(GPIO_NUM_4, DSHOT300_BIDIRECTIONAL);
TickType_t xLastWakeTime;
log_entry_t log_buf[10000] = {};
size_t log_pos = 0;

void log(uint16_t throttle, uint32_t erpm)
{
    log_buf[log_pos] = {
        .throttle = throttle,
        .erpm = erpm,
        .tick = xTaskGetTickCount(),
    };
    log_pos++;
}

void clearLog()
{
    // memset(log_buf, 0, sizeof(log_buf));
    log_pos = 0;
}

void printLog()
{
    auto rpmRatio = DShotRMT::getErpmToRpmRatio(14);
    for (size_t i = 0; i < log_pos; i++)
    {
        log_entry_t cur = log_buf[i];
        float throttle = ((float)cur.throttle - DSHOT_THROTTLE_MIN) / (float)DSHOT_THROTTLE_RANGE;
        float rpm = cur.erpm * rpmRatio;
        ESP_LOGI(TAG, "TICK: %lu, Throttle %%: %5.1f, RPM: %5.0f", cur.tick, throttle * 100, rpm);
    }
}

void rampThrottle(int start, int stop, int step)
{
    if (step == 0)
        return;

    for (int i = start; step > 0 ? i < stop : i > stop; i += step)
    {
        motor.sendThrottle(i);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
    motor.sendThrottle(stop);
}

void rampThrottle(int runs)
{
    xLastWakeTime = xTaskGetTickCount();
    int taskCounter = 0;
    int rampMax = DSHOT_THROTTLE_RANGE * 0.3;

    ESP_LOGI(TAG, "Ramping throttle...");
    while (true)
    {
        // Ramp
        rampThrottle(DSHOT_THROTTLE_MIN + 50, rampMax, 1);
        rampThrottle(rampMax, DSHOT_THROTTLE_MIN + 50, -1);

        taskCounter++;
        if (taskCounter >= runs)
            break;
    }

    ESP_LOGI(TAG, "Done!");
}

void stepResponse()
{
    xLastWakeTime = xTaskGetTickCount();
    int taskCounter = 0;
    uint32_t erpm = 0;
    
    ESP_LOGI(TAG, "Running throttle/RPM step response...");

    uint16_t throttle = 200;
    while (true)
    {
        erpm = motor.sendThrottle(throttle);
        log(throttle, erpm);

        taskCounter++;
        if (taskCounter >= 150)
            break;

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }

    taskCounter = 0;
    throttle = 400;
    while (true)
    {
        erpm = motor.sendThrottle(throttle);
        log(throttle, erpm);

        taskCounter++;
        if (taskCounter >= 150)
            break;

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }

    taskCounter = 0;
    throttle = 200;
    while (true)
    {
        erpm = motor.sendThrottle(throttle);
        log(throttle, erpm);

        taskCounter++;
        if (taskCounter >= 150)
            break;

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }

    motor.sendThrottle(0);
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Done!");
    printLog();
    clearLog();
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initializing DShot RMT...");
    motor.begin();

    xLastWakeTime = xTaskGetTickCount();

    // rampThrottle(2);

    stepResponse();

    ESP_LOGW(TAG, "Exiting");
}
