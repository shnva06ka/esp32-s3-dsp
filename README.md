# ESP32-S3-DSP

Real-time stereo digital audio processor for the ESP32-S3 with support for multiple inputs and basic effects.

---

## Architecture

```
I2S ADC
Bluetooth A2DP -> Input selector -> DSP engine -> I2S DAC
SlimProto
```

> [!NOTE]
> Optional ESP32-WROOM is used here as A2DP receiver, as the ESP32-S3 itself does not support Bluetooth Classic. 
> Any alternative with I2S output can be substituted.

SlimProto can serve any music player with support for output to Logitech Media Server (LMS).

> [!CAUTION]
> Disable FLAC compression if using SlimProto. Only raw PCM is supported.

Supports standard I2S DACs and I2S ADCs; the board has been tested with PCM1808 and PCM5102A.

---

## DSP Pipeline

Highly modular, and optional effect modules; supports real time processing order rearrangement.

- **Parametric EQ**: Up to 10 filters: Peaking, Low/High Shelf, Low/High Pass with 1st to 4th order slopes.
- **Speakers Correction**: Subsonic high-pass 20 to 100 Hz for subwoofer protection, L/R balance, per channel delay, phase shift.
- **Compressor**: RMS compressor with threshold, ratio, attack, release, and makeup gain.
- **Limiter**: Soft-knee peak ceiling limiter to prevent digital clipping.

---

### Core Allocation (ESP32-S3)

- **Core 1 (Audio Engine)**: FreeRTOS, DMA at 48 kHz 32-bit, 256 frame buffers DSP
- **Core 0 (Network and Control)**: Wi-Fi WebUI, UART bridge to ESP32-WROOM, SlimProto client

---

## Audio Hardware Wiring

ESP32-S3 interfaces with external audio converters via I2S0 master mode.

| Signal | ESP32-S3 | Connected Hardware | Function        |
|--------|----------|--------------------|-----------------|
| MCLK   | GPIO 4   | PCM1808 ADC        | Master Clock    |
| BCK    | GPIO 5   | PCM1808 & PCM5102A | I2S0 Bit Clock  |
| WS     | GPIO 6   | PCM1808 & PCM5102A | I2S0 Frame Sync |
| DIN    | GPIO 15  | PCM1808 DOUT       | ADC Line in     |
| DOUT   | GPIO 7   | PCM5102A DIN       | DAC Line out    |

> [!NOTE]
> Set MD0/MD1 for Slave mode. Requires MCLK on GPIO 4.
> Tie SCK to GND to enable internal PLL.

---

## Inter-Board Wiring (ESP32-S3 to ESP32-WROOM)

| Signal | ESP32-WROOM | ESP32-S3 | Description             |
|--------|-------------|----------|-------------------------|
| GND    | GND         | GND      | Reference GND           |
| BCK    | GPIO 22     | GPIO 16  | I2S1 Bit Clock          |
| WS     | GPIO 21     | GPIO 17  | I2S1 Word Select, LRCK  |
| DIN    | GPIO 23     | GPIO 18  | I2S1 Audio Data         |
| TX     | GPIO 19     | GPIO 44  | Metadata (WROOM to S3)  |
| RX     | GPIO 18     | GPIO 43  | Commands (S3 to WROOM)  |

---

## Build and Flash

The project builds via PlatformIO:

```bash
# Flash ESP32-S3
pio run -e yd_esp32s3_n16r8 -t upload

# Flash ESP32-WROOM Bluetooth bridge
pio run -e wroom_bt -t upload

# Monitor serial debug
pio device monitor -e yd_esp32s3_n16r8
```

---

## Wi-Fi Configuration

Configure Wi-Fi credentials in `src/wifi_credentials.h`
Example:
```cpp
#pragma once
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourNetworkPassword"
```
IP address printed on serial monitor.
If it can't connect (or there are no credentials), starts fallback SoftAP:
- **SSID**: `ESP32-S3-DSP`
- **URL**: `http://192.168.4.1`
