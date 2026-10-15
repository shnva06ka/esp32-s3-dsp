# ESP32-S3-DSP

Real-time stereo DSP processor built on an ESP32-S3 (YD-ESP32-S3, 16 MB Flash, 8 MB PSRAM).

## Hardware

| Signal | GPIO | IC |
|--------|------|----|
| MCLK   | 4    | both |
| BCK    | 5    | both |
| WS     | 6    | both |
| DOUT   | 7    | PCM5102A DAC (ESP → DAC) |
| DIN    | 15   | PCM1808 ADC (ADC → ESP)  |

- **ADC**: PCM1808 — 24-bit, 48 kHz, I²S, requires MCLK = 256 × Fs (12.288 MHz)
- **DAC**: PCM5102A — 24-bit, 48 kHz, I²S, no MCLK needed (SCK-less mode)

## Project Structure

```
ESP32_DSP/
├── platformio.ini            # Build configuration
├── partitions_16MB.csv       # Partition table (dual OTA + SPIFFS)
├── data/
│   └── index.html            # Web UI (served from SPIFFS)
└── src/
    ├── main.cpp              # Entry: creates Core 0 / Core 1 tasks
    ├── audio_config.h        # Compile-time constants (pins, sample rate, …)
    ├── param_store.h         # Thread-safe shared parameter store (mutex)
    ├── i2s_audio.h/.cpp      # Full-duplex I2S driver (PCM1808 + PCM5102A)
    ├── biquad_eq.h           # 10-band Parametric EQ (esp-dsp biquads)
    ├── compander.h           # Compressor
    ├── limiter.h             # Limiter
    ├── dsp_pipeline.h        # Pipeline: EQ → Compressor → Limiter
    └── web_ui.h              # Async WebServer + REST API
```

## DSP Pipeline (Core 1)

```
ADC (PCM1808) ──I2S DMA──▶ int32 → float
                                │
                         ┌──── ▼ ────┐
                         │   10-Band │
                         │   Biquad  │  ← /api/eq
                         │    EQ     │
                         └──── ▼ ────┘
                         ┌──── ▼ ────┐
                         │  Stereo   │
                         │   RMS     │  ← /api/compander
                         │ Compander │
                         └──── ▼ ────┘
                         ┌──── ▼ ────┐
                         │   Soft    │
                         │  Limiter  │  ← /api/limiter
                         └──── ▼ ────┘
                                │
                       float → int32 ──I2S DMA──▶ DAC (PCM5102A)
```

## Web UI (Core 0)

Navigate to the board's IP address in any browser.

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serve `index.html` |
| `/api/params` | GET | Dump all parameters as JSON |
| `/api/eq` | POST | Update one EQ band `{band, freq, gain, q, enabled}` |
| `/api/compander` | POST | Update compander `{threshold, ratio, attackMs, releaseMs, makeup, enabled}` |
| `/api/limiter` | POST | Update limiter `{ceiling, knee, enabled}` |
| `/api/meters` | GET | RMS level meters (stub) |

Wi-Fi credentials default to `YourSSID` / `YourPassword`.  
Override with `-DWIFI_SSID=…` and `-DWIFI_PASS=…` in `build_flags`, or implement NVS loading.

Falls back to SoftAP `ESP32-DSP` / `dspcontrol` if station Wi-Fi fails.

## Building & Flashing

```bash
# Build
pio run -e yd_esp32s3_n16r8

# Upload firmware
pio run -e yd_esp32s3_n16r8 -t upload

# Upload web UI to SPIFFS
pio run -e yd_esp32s3_n16r8 -t uploadfs

# Monitor serial
pio device monitor
```

## EQ Default Bands (ISO 266)

| Band | Freq | Default Gain | Default Q |
|------|------|-------------|-----------|
| 0 | 31.5 Hz | 0 dB | 0.707 |
| 1 | 63 Hz | 0 dB | 0.707 |
| 2 | 125 Hz | 0 dB | 0.707 |
| 3 | 250 Hz | 0 dB | 0.707 |
| 4 | 500 Hz | 0 dB | 0.707 |
| 5 | 1 kHz | 0 dB | 0.707 |
| 6 | 2 kHz | 0 dB | 0.707 |
| 7 | 4 kHz | 0 dB | 0.707 |
| 8 | 8 kHz | 0 dB | 0.707 |
| 9 | 16 kHz | 0 dB | 0.707 |

## Compander Defaults

| Parameter | Default |
|-----------|---------|
| Threshold | −18 dBFS |
| Ratio | 4:1 |
| Attack | 5 ms |
| Release | 150 ms |
| Makeup Gain | 0 dB |

## Limiter Defaults

| Parameter | Default |
|-----------|---------|
| Ceiling | −0.5 dBFS |
| Knee | 3 dB |

## Extending

- **NVS persistence**: implement `ParamStore::save()` / `begin()` using `Preferences`
- **WebSocket meters**: add `AsyncWebSocket` at `/ws/meters` and push RMS values from a timer task
- **OTA updates**: add ArduinoOTA or ElegantOTA in `WebUI::begin()`
- **More EQ types**: switch to high-shelf/low-shelf/notch per band by changing `_calcCoeffs()`
- **SIMD optimisation**: wrap the compander gain loop with `dsps_mulc_f32()` for throughput

