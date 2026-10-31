# ESP32-S3-DSP

Real-time stereo audio processor built on the ESP32-S3, controlled from a web interface.

## Features

- Three inputs: Line In, Bluetooth, Squeezebox
- Fixed 44.1 kHz sample rate end to end, with no resampling
- Parametric EQ with up to 10 bands (Peaking, Shelf, Low Pass, High Pass, up to 4th order)
- Speaker correction: subsonic filter, balance, per-channel delay, phase
- Compressor and soft-knee limiter
- Adjustable processing order
- Web UI with live EQ graph, presets, and Wi-Fi setup
- OTA firmware updates at `/update`

## Architecture

```
Line In (I2S ADC) --\
Bluetooth A2DP -------> Input selector -> DSP -> I2S DAC
Squeezebox ---------/
```

- **ESP32-S3**: audio engine, DSP, web UI, and Squeezebox client
- **ESP32-WROOM** (optional): Bluetooth A2DP receiver, since the S3 has no Bluetooth Classic. Any I2S source can replace it.
- **Squeezebox**: works with Logitech Media Server and compatible players such as Roon. Supports FLAC, WAV, AIFF, and PCM. The server converts other rates to 44.1 kHz.

Tested with PCM1808 (ADC) and PCM5102A (DAC).

## Wiring

ESP32-S3 to audio converters (I2S0):

| Signal | GPIO | Connected to       |
|--------|------|--------------------|
| MCLK   | 4    | PCM1808            |
| BCK    | 5    | PCM1808 and PCM5102A |
| WS     | 6    | PCM1808 and PCM5102A |
| DIN    | 15   | PCM1808 DOUT       |
| DOUT   | 7    | PCM5102A DIN       |

Set the PCM1808 to slave mode and tie PCM5102A SCK to GND.

ESP32-WROOM to ESP32-S3:

| Signal | WROOM   | S3      |
|--------|---------|---------|
| GND    | GND     | GND     |
| BCK    | GPIO 22 | GPIO 16 |
| WS     | GPIO 21 | GPIO 17 |
| DATA   | GPIO 23 | GPIO 18 |

## Build

```bash
pio run -e yd_esp32s3_n16r8 -t upload   # ESP32-S3
pio run -e wroom_bt -t upload           # ESP32-WROOM Bluetooth bridge
pio device monitor -e yd_esp32s3_n16r8  # serial monitor
```

The streaming buffer needs PSRAM. Check the boot log for OPI PSRAM initialization.

## Wi-Fi

Set credentials in `src/wifi_credentials.h`, or use the Wi-Fi setup card in the web UI:

```cpp
#pragma once
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourNetworkPassword"
```

Once connected, open `http://dsp.local`. If no network is available, the board starts an access point named `ESP32-S3-DSP` at `http://192.168.4.1`.
