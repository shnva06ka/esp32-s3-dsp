# ESP32-S3-DSP

Real-time stereo audio processor built on the ESP32-S3, controlled from a web interface.

## Features

- Three inputs: Line In, Bluetooth, Squeezebox
- Fixed 96 kHz sample rate for DSP and I2S, with on-device resampling for lower rates
- Parametric EQ with up to 10 bands (Peaking, Shelf, Low Pass, High Pass, up to 4th order)
- Speaker correction: subsonic filter, balance, per-channel delay, phase
- Compressor and soft-knee limiter
- Adjustable processing order
- Web UI with live EQ graph, presets, and Wi-Fi setup
- OTA firmware updates at `/update`

## Hardware Requirements

- **ESP32-S3-N16R8** (16 MB Flash, 8 MB Octal OPI PSRAM)
- **Audio Converters**: PCM1808 (ADC) and PCM5102A (DAC), or any standard I2S DAC/ADC supporting 256fs MCLK (24.576 MHz generated via ESP32-S3 APLL).
- **Bluetooth Bridge (Optional)**: ESP32-WROOM (or any classic ESP32 module) running the included bridge firmware.

> [!WARNING]
> The ESP32-S3-N16R8 is strictly required, modules without PSRAM or with Quad PSRAM (N8R2, N16R2) will panic on boot.


## Getting Started

### Wi-Fi Setup

Set credentials in `src/wifi_credentials.h`, or use the Wi-Fi setup card in the web UI:

```cpp
#pragma once
#define WIFI_SSID "YourNetworkName"
#define WIFI_PASS "YourNetworkPassword"
```

Once connected, open `http://dsp.local`. 
If no network is available, or credentials not configured,  
the board starts an access point named `ESP32-S3-DSP` at `http://192.168.4.1`.  
Connect to this access point and open `http://dsp.local` to access the web UI.  

### Build & Flash

```bash
pio run -e yd_esp32s3_n16r8 -t upload   # ESP32-S3
pio run -e wroom_bt -t upload           # ESP32-WROOM Bluetooth bridge
pio device monitor -e yd_esp32s3_n16r8  # serial monitor
```

## Architecture

```
Line In (I2S ADC) --\
Bluetooth A2DP -------> Input selector -> DSP -> I2S DAC
Squeezebox ---------/
```

- **ESP32-S3**: audio engine, DSP, web UI, and Squeezebox client
- **ESP32-WROOM** (optional): Bluetooth A2DP receiver, since the S3 has no Bluetooth Classic. Any I2S source can replace it.
- **Squeezebox**: works with Logitech Media Server and compatible players such as Roon.  
    Supports FLAC, WAV, AIFF, and PCM up to 96 kHz; lower rates (44.1, 48, 88.2 kHz) are resampled to 96 kHz.
Tested with PCM1808 (ADC) and PCM5102A (DAC).

## Wiring

ESP32-S3 to audio converters (I2S0):

| Signal | GPIO | Connected to       |
|--------|------|--------------------|
| MCLK   | 4    | PCM1808            |
| BCK    | 5    | PCM1808 & PCM5102A |
| WS     | 6    | PCM1808 & PCM5102A |
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

## API

The board exposes a REST API and a WebSocket at `/ws`:

| Endpoint | Method | Description |
|---|---|---|
| `/api/params` | GET | DSP state, active input, stream info, and system diagnostics |
| `/api/resampler` | GET | Resampler latency, active ratio, CPU load, and buffer stats |
| `/api/resampler/validate` | POST | Run resampler benchmark across standard sample rates |
| `/api/resampler/log` | POST | Enable or disable periodic serial telemetry (`enable=1` or `0`) |
| `/api/wifi` | GET, POST | Check connection status; configure SSID and password |
| `/api/logs` | GET | Retrieve recent console logs as plain text |
| `/api/reboot` | POST | Reboot the ESP32-S3 |
| `/console` | GET | Web-based live console |
| `/update` | GET, POST | Firmware upload page and OTA binary endpoint |
| `/ws` | WS | Real-time parameter sync and live 1 Hz system telemetry |

## License & Credits

This project is licensed under the **GNU General Public License v3.0 (GPLv3)**. See the [LICENSE](LICENSE) file for the full license text.

### Third-Party Acknowledgements

- **Squeezelite**: (c) 2012–2015 Adrian Smith, 2015–2021 Ralph Irving (GPLv3).
- **squeezelite-esp32**: (c) Philippe G. / sle118 (GPLv3/MIT).
- **dr_flac**: (c) 2023 David Reid (Public Domain / MIT-0).
- **ESP32-A2DP**: (c) Phil Schatzmann (MIT).
- **ESPAsyncWebServer & AsyncTCP**: (c) Mathieu Carbou (LGPLv3).
- **ArduinoJson**: (c) Benoit Blanchon (MIT).
