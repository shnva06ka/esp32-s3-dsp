# ESP32-S3-DSP

Real-time stereo digital audio processor on dual ESP32 microcontrollers. Combines an ESP32-S3 audio engine with an ESP32-WROOM Bluetooth co-processor, PCM1808 ADC, PCM5102A DAC, and responsive WebUI.

---

## Architecture

```
                    ESP32-WROOM (Co-Processor)
                 +-------------------------------+
                 |  Bluetooth A2DP Audio Sink    |
Phone / PC ----->|  AVRCP Metadata and Transport |
 (Bluetooth)     +---------------+---------------+
                                 |
                 I2S (Audio)     |  UART (Metadata and Controls)
                 GPIO 22/21/23   |  GPIO 19/18
                                 v
                     ESP32-S3 (Audio Processor)
                 +-------------------------------+
AUX Line In ---->|  Input Selector:              |
 (PCM1808 ADC)   |   1. AUX Line In (PCM1808)    |
Wi-Fi Stream --->|   2. Bluetooth A2DP           |
(LMS / Roon)     |   3. Wi-Fi Lossless (SlimProto|
                 +-------------------------------+
                 |  Core 1: DSP Pipeline         |
                 |   - Preamp Gain Stage         |
                 |   - Parametric Biquad EQ      |
                 |   - Subsonic High-Pass Filter |
                 |   - Stereo RMS Compressor     |
                 |   - Soft-Knee Peak Limiter    |
                 +-------------------------------+
                 |  Core 0: Network and WebUI    |
                 |   - AsyncWebServer (PROGMEM)  |
                 |   - Bidirectional WebSocket   |
                 |   - Squeezelite SlimProto     |
                 +---------------+---------------+
                                 | I2S Audio
                                 v
                        PCM5102A Stereo DAC
                                 |
                                 v
                         Analog Audio Out
```

### Core Allocation (ESP32-S3)

- **Core 1 (Audio Engine)**: FreeRTOS priority 24, DMA transfers at 48 kHz stereo 32-bit, zero-latency DSP in-place processing.
- **Core 0 (Network and Control)**: AsyncWebServer on port 80, WebSocket `/ws`, UART bridge to ESP32-WROOM, Wi-Fi Station and SoftAP fallback.

---

## Inter-Board Wiring (ESP32-S3 to ESP32-WROOM)

| Signal | ESP32-WROOM | ESP32-S3 | Description |
|--------|-------------|----------|-------------|
| GND    | GND         | GND      | Common reference ground |
| BCK    | GPIO 22     | GPIO 16  | I2S1 Bit Clock |
| WS     | GPIO 21     | GPIO 17  | I2S1 Word Select / LRCK |
| DIN    | GPIO 23     | GPIO 18  | I2S1 Audio Data |
| TX     | GPIO 19     | GPIO 44  | UART Metadata (WROOM to S3) |
| RX     | GPIO 18     | GPIO 43  | UART Commands (S3 to WROOM) |

---

## Audio Hardware Wiring

ESP32-S3 interfaces with external audio converters via I2S0 master mode.

| Signal | ESP32-S3 | Connected Hardware | Function |
|--------|----------|--------------------|----------|
| MCLK   | GPIO 4   | PCM1808 ADC        | 12.288 MHz Master Clock (256 * Fs) |
| BCK    | GPIO 5   | PCM1808 and PCM5102A | I2S0 Bit Clock |
| WS     | GPIO 6   | PCM1808 and PCM5102A | I2S0 Frame Sync |
| DIN    | GPIO 15  | PCM1808 DOUT       | ADC Line-In capture |
| DOUT   | GPIO 7   | PCM5102A DIN       | Processed audio stream to DAC |

- **PCM1808 ADC**: Set MD0/MD1 for Slave mode. Requires MCLK on GPIO 4.
- **PCM5102A DAC**: Tie SCK to GND to enable internal PLL.

---

## DSP Pipeline

Audio executes on Core 1 at 48 kHz (256 samples/buffer):

```
Input (AUX / BT / Wi-Fi) -> Preamp -> Parametric EQ -> Subsonic Filter -> Compressor -> Limiter -> Output
```

- **Preamp**: -18 dB to +18 dB gain.
- **Parametric EQ**: Up to 10 biquad filters (Peaking, Low/High Shelf, Low/High Pass).
- **Subsonic Filter**: High-pass biquad (20 to 100 Hz, 1st to 4th order: 6, 12, 18, 24 dB/oct slopes) to protect subwoofer drivers.
- **Compressor**: RMS dynamic range compander with threshold, ratio, attack, release, and makeup gain.
- **Limiter**: Soft-knee peak ceiling limiter to prevent digital clipping.

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

Configure Wi-Fi credentials in `src/wifi_credentials.h`:

```cpp
#pragma once
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourNetworkPassword"
```

If unconnected, the board starts a fallback SoftAP:
- **SSID**: `ESP32-DSP`
- **Password**: `dspcontrol`
- **URL**: `http://192.168.4.1`
