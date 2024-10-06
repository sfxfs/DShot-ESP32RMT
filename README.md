# ESP32 DShot Library utilizing RMT
The library provides a convenient way of generating and reading DShot signals using the RMT peripheral on the ESP32 platform, supporting all protocol speeds: DSHOT150, DSHOT300, DSHOT600, and DSHOT1200.

## The DShot Protocol
The DSHOT protocol consists of transmitting 16-bit packets to the ESCs: 11-bit throttle value, 1-bit to request telemetry and a 4-bit checksum.

| DSHOT | Bitrate   | TH1   | TH0    | Bit Time µs | Frame Time µs |
|-------|------------|-------|--------|------------|---------------|
| 150   | 150kbit/s  | 5.00  | 2.50   | 6.67       | 106.72        |
| 300   | 300kbit/s  | 2.50  | 1.25   | 3.33       | 53.28         |
| 600   | 600kbit/s  | 1.25  | 0.625  | 1.67       | 26.72         |
| 1200  | 1200kbit/s | 0.625 | 0.313  | 0.83       | 13.28         |

### Bidirectional DSHOT
When bidirectional DHSOT is enabled, for each frame sent to the ESC a frame with eRPM telemetry data is returned (on the same line, not the additional telemetry line), effectively halving the amount of frames you can send per second. You need to keep this in mind, especially when running higher PID frequencies.

> Bidirectional DSHOT only works with DSHOT 300 and up

### eRPM Telemetry (from ESC)
The ESCs report eRPM (electrical rpm). This must be converted to RPM using the number of magnets of the motors (those on the bell of the motor).

## Using RMT on ESP32
The RMT (Remote Control) is a peripheral designed to generate accurate and stable signals to control external devices such as LEDs, motors, and other peripherals. It is well suited for generating the DShot signals in a high-performance and accurate way on the ESP32 platform.

### Advantages of using RMT
- Generates accurate signals
- Supports programmable timing
- Offloads the CPU

## References
- [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
- [DSHOT Betaflight docs](https://betaflight.com/docs/development/dshot)
- [ESP32 RMT Programming Guide](https://docs.espressif.com/projects/esp-idf/en/v5.1.4/esp32/api-reference/peripherals/rmt.html)
