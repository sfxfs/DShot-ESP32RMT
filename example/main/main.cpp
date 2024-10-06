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

DShotRMT motor(GPIO_NUM_4, DSHOT600_BIDIRECTIONAL);
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
    // Get the ratio to convert eRPM to real RPM.
    // !! Use actual pole count (number of magnes on the bell) of your motor !!
    auto rpmRatio = DShotRMT::getErpmToRpmRatio(14);

    for (size_t i = 0; i < log_pos; i++)
    {
        log_entry_t cur = log_buf[i];
        float throttle = ((float)cur.throttle - DSHOT_THROTTLE_MIN) / (float)DSHOT_THROTTLE_RANGE;
        float rpm = INVALID_TELEMETRY_VALUE;
        if (cur.erpm != INVALID_TELEMETRY_VALUE)
            rpm = cur.erpm * rpmRatio;
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

    uint16_t throttle_low = 200;
    uint16_t throttle_high = 400;
    int loops = 150;
    TickType_t loop_delay = pdMS_TO_TICKS(1);

    // eRPM can be consumed in three ways, demonstrated below

    while (true)
    {
        // 1. First method to get eRPM - when the shortest possible delay in RPM reading is required:
        // - Send throttle, which occurs non-blocking on hardware, then
        // - Immediately wait for the telemetry reponse, which is a blocking "busy-wait" operation.

        motor.sendThrottle(throttle_low);
        motor.waitForErpm(erpm);
        log(throttle_low, erpm);

        xTaskDelayUntil(&xLastWakeTime, loop_delay);

        taskCounter++;
        if (taskCounter >= loops)
            break;
    }

    taskCounter = 0;
    while (true)
    {
        // 2. Second method to get eRPM - when the most recent telemetry is still desired, but not as time critical
        // - Send throttle, which occurs non-blocking on hardware, then
        // - Do some other work, then
        // - Wait for the telemetry reponse, which is a blocking "busy-wait" operation.
        //   If waiting is started after the telemetry response already has been recieved
        //   the function will return immediately.

        motor.sendThrottle(throttle_high);

        // Some other work...

        motor.waitForErpm(erpm);
        log(throttle_high, erpm);

        xTaskDelayUntil(&xLastWakeTime, loop_delay);

        taskCounter++;
        if (taskCounter >= loops)
            break;
    }

    taskCounter = 0;
    while (true)
    {
        // 3. Third method to get eRPM - when lock-step is not required
        // - Send throttle, which occurs non-blocking on hardware, then
        // - Get the last received RPM (non-blocking)

        motor.sendThrottle(throttle_low);

        xTaskDelayUntil(&xLastWakeTime, loop_delay);

        erpm = motor.getErpm();
        log(throttle_low, erpm);

        taskCounter++;
        if (taskCounter >= loops)
            break;
    }

    motor.sendThrottle(0);

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
