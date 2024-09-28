#include "DShotRMT.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <esp_log.h>

#include "dshot_rmt_encoder.h"

static const char *TAG = "DShotRMT";

// Constructor that takes gpio and rmtChannel as arguments
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_e dshot_mode)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    bool bidirectional = dshot_mode >= DSHOT300_BIDIRECTIONAL && dshot_mode <= DSHOT1200_BIDIRECTIONAL;

    ESP_LOGI(TAG, "Create RMT RX channel");
    const rmt_rx_channel_config_t rmt_rx_channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = MAX_BLOCKS};
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_channel_config, &rmt_rx_channel));

    ESP_LOGI(TAG, "Create RMT TX channel");
    const rmt_tx_channel_config_t rmt_tx_channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT, // a clock that can provide needed resolution
        .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = MAX_BLOCKS,
        .trans_queue_depth = 2, // set the number of transactions that can be pending in the background
        .flags = {
            .invert_out = bidirectional,
            .with_dma = true,
        }};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_tx_channel_config, &rmt_tx_channel));

    ESP_LOGI(TAG, "Install Dshot RMT encoder");
    uint32_t baudrate = 0;
    uint32_t post_delay_us = 0;
    switch (dshot_mode)
    {
    case DSHOT150:
        baudrate = 150000;
        post_delay_us = 50;
        break;
    case DSHOT300:
    case DSHOT300_BIDIRECTIONAL:
        baudrate = 300000;
        post_delay_us = 25;
        break;
    case DSHOT600:
    case DSHOT600_BIDIRECTIONAL:
        baudrate = 600000;
        post_delay_us = 20;
        break;
    case DSHOT1200:
    case DSHOT1200_BIDIRECTIONAL:
        baudrate = 1200000;
        post_delay_us = 20;
        break;
    default:
        break;
    }

    encoder_config = {
        .resolution = DSHOT_RMT_RESOLUTION_HZ,
        .baud_rate = baudrate,
        .bidirectional = bidirectional,
        .post_delay_us = post_delay_us, // extra delay between each frame
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    // Initialize sending structs
    tx_config = {
        .loop_count = 0,
    };
    throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };
}

DShotRMT::~DShotRMT()
{
    // Uninstall the RMT driver
    if (dshot_encoder)
    {
        rmt_del_encoder(dshot_encoder);
    }

    if (rmt_rx_channel)
    {
        if (enabled)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_rx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_rx_channel));
    }

    if (rmt_tx_channel)
    {
        if (enabled)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_tx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_tx_channel));
    }

    enabled = false;
}

void DShotRMT::begin()
{
    ESP_LOGI(TAG, "Enable RMT RX channel");
    ESP_ERROR_CHECK(rmt_enable(rmt_rx_channel));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(rmt_tx_channel));
    enabled = true;

    ESP_LOGI(TAG, "Resetting and Arming ESC...");
    sendTicks(0, pdMS_TO_TICKS(DSHOT_ARM_DELAY));

    ESP_LOGI(TAG, "Done!");
}

void DShotRMT::sendThrottle(uint16_t throttle_value)
{
    if (throttle_value > DSHOT_THROTTLE_MAX)
    {
        throttle_value = DSHOT_THROTTLE_MAX;
    }
    else if (throttle_value < DSHOT_THROTTLE_MIN)
    {
        throttle_value = DSHOT_THROTTLE_MIN;
    }

    send(throttle_value);
}

void DShotRMT::send(uint16_t value, int loop_count)
{
    if (!enabled)
        return;

    throttle.throttle = value;

    int prevLoopCount = tx_config.loop_count;
    tx_config.loop_count = loop_count;

    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_channel, dshot_encoder, &throttle, sizeof(throttle), &tx_config));

    if (prevLoopCount == -1)
    {
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(rmt_tx_channel));
        ESP_ERROR_CHECK(rmt_enable(rmt_tx_channel));
    }
}

void DShotRMT::sendTicks(uint16_t value, TickType_t ticks)
{
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_channel, ticks));

    send(value, -1);

    vTaskDelay(ticks);

    send(value, 0);
}
