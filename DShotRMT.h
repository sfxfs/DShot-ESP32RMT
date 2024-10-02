#pragma once

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "dshot_rmt_encoder.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_RMT_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_RMT_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define MAX_BLOCKS 64
#else
#define MAX_BLOCKS 48
#endif

// DSHOT Timings
#define DSHOT_ARM_DELAY 3200

// Constants related to the DShot protocol
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_THROTTLE_RANGE (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN)
static constexpr uint32_t INVALID_TELEMETRY_VALUE = 0xffff;

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200,
    DSHOT300_BIDIRECTIONAL,
    DSHOT600_BIDIRECTIONAL,
    DSHOT1200_BIDIRECTIONAL
} dshot_mode_t;

// The main DShotRMT class
class DShotRMT
{
public:
    // Constructor for the DShotRMT class with a given DShot mode
    DShotRMT(gpio_num_t gpio, dshot_mode_t dshot_mode);

    // Destructor for the DShotRMT class
    ~DShotRMT();

    // The begin() function enalbes the DShotRMT class
    void begin();

    // The sendThrottle() function sends a DShot packet with a given
    // throttle value (between 48 and 2047)
    uint32_t sendThrottle(uint16_t throttle_value);

    static float getErpmToRpmRatio(int poles)
    {
        static constexpr float ERPM_PER_LSB = 100.0f;
        return ERPM_PER_LSB / (poles / 2.0f);
    }

private:
    gpio_num_t gpio_num;
    rmt_channel_handle_t rmt_rx_channel = NULL;
    rmt_channel_handle_t rmt_tx_channel = NULL;
    rmt_encoder_handle_t dshot_encoder = NULL;
    rmt_receive_config_t rx_config;
    rmt_transmit_config_t tx_config;
    QueueHandle_t rx_queue;
    dshot_rmt_throttle_t throttle;
    dshot_rmt_encoder_config_t encoder_config;
    bool enabled = false;
    rmt_symbol_word_t rx_buf[MAX_BLOCKS];
    bool mode = false;
    bool is_bidirectional = false;
    uint16_t telemetry_bit_len_ticks;
    uint32_t telemetry_errors = 0;

    void send(uint16_t value);
    void sendTicks(uint16_t value, TickType_t ticks);
    uint32_t receive();
    void modeTx();
    void modeRx();
    void enableRx();
    void disableRx();
    static bool rxDoneCallback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    static bool txDoneCallback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data);
};
