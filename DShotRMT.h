#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <driver/rmt_tx.h>

#include "dshot_rmt_encoder.h"

#if CONFIG_IDF_TARGET_ESP32H2
#define DSHOT_RMT_RESOLUTION_HZ 32000000 // 32MHz resolution, DSHot protocol needs a relative high resolution
#else
#define DSHOT_RMT_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#endif

// Constants related to the DShot protocol
constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// The main DShotRMT class
class DShotRMT
{
public:
    // Constructor for the DShotRMT class
    DShotRMT(gpio_num_t gpio, dshot_mode_e dshot_mode);
    // DShotRMT(uint8_t pin, uint8_t channel);
    // DShotRMT(uint8_t pin);

    // Destructor for the DShotRMT class
    ~DShotRMT();

    // The begin() function initializes the DShotRMT class with
    // a given DShot mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)
    // and a bidirectional flag. It returns a boolean value
    // indicating whether or not the initialization was successful.
    void begin(bool is_bidirectional = false);

    // The sendThrottleValue() function sends a DShot packet with a given
    // throttle value (between 48 and 2047)
    void sendThrottleValue(uint16_t throttle_value);

private:
    // rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH]; // An array of RMT items used to send a DShot packet.
    rmt_tx_channel_config_t rmt_tx_channel_config; // The RMT configuration used for sending DShot packets.
    rmt_channel_handle_t rmt_tx_channel = NULL;
    rmt_encoder_handle_t dshot_encoder = NULL;
    rmt_transmit_config_t tx_config;
    dshot_rmt_throttle_t throttle;
    dshot_rmt_encoder_config_t encoder_config;
    bool enabled = false;
};

#endif
