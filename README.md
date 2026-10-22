# ESP32-S3-DSP

Real-time stereo digital audio processor on dual ESP32 microcontrollers. Combines an ESP32-S3 audio engine with an ESP32-WROOM Bluetooth co-processor, PCM1808 ADC, PCM5102A DAC, and WebUI.

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
                 |   3. Wi-Fi (SlimProto)        |
                 +-------------------------------+
                 |  Core 1: DSP Pipeline         |
                 |   - Parametric EQ + Preamp    |
                 |   - Speakers Correction       |
                 |   - RMS Compressor            |
                 |   - Soft-Knee Limiter         |
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

- **Core 1 (Audio Engine)**: FreeRTOS priority 24, DMA at 48 kHz stereo 32-bit, DSP runs in place on 256 frame buffers (~5.3 ms).
- **Core 0 (Network and Control)**: AsyncWebServer on port 80, WebSocket `/ws`, UART bridge to ESP32-WROOM, SlimProto client (Roon / LMS), Wi-Fi Station and SoftAP fallback.

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
Input (AUX / BT / Wi-Fi) -> EQ (with Preamp) -> Speakers Correction -> Compressor -> Limiter -> Output
```

That's the default order. Modules are independent and the order can be changed in the WebUI. Everything starts OFF on a fresh board, settings are saved to flash.

- **Parametric EQ**: Up to 10 biquad filters (Peaking, Low/High Shelf, Low/High Pass with 1st to 4th order slopes) and a -18 dB to +18 dB preamp.
- **Speakers Correction**: Subsonic high-pass (20 to 100 Hz, 1st to 4th order: 6, 12, 18, 24 dB/oct) to protect subwoofer drivers, plus L/R balance, per channel delay (0 to 20 ms) and phase shift (0 to 180 deg, 45 deg steps).
- **Compressor**: RMS compressor with threshold, ratio, attack, release, and makeup gain.
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

Configure Wi-Fi credentials in `src/wifi_credentials.h` (copy `src/wifi_credentials.example.h`, it's gitignored):

```cpp
#pragma once
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourNetworkPassword"
```

If it can't connect (or there are no credentials), the board starts a fallback SoftAP:
- **SSID**: `ESP32-S3-DSP`
- **Password**: none (open network)
- **URL**: `http://192.168.4.1`
