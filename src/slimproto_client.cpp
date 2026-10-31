#include "slimproto_client.h"
#include "squeezelite.h"
#include "wireless_console.h"
#include <WiFi.h>
#include <esp_wifi.h>

#define Serial Console

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;

SlimprotoClient::SlimprotoClient(ParamStore& params)
    : _params(params)
{
}

SlimprotoClient::~SlimprotoClient() {
    stop();
}

bool SlimprotoClient::begin() {
    if (_running) return true;

    Serial.println("[SLIMPROTO] Starting Squeezelite streaming pipeline...");

    uint8_t mac[6];
    WiFi.macAddress(mac);

    stream_init(lINFO, STREAMBUF_SIZE);
    decode_init(lINFO, "pcm,flc", "");

    slimproto_start(lINFO, NULL, mac, "ESP32-S3-DSP", 44100);

    _running = true;
    xTaskCreatePinnedToCore(
        _telemetryTaskTrampoline,
        "sl_telemetry",
        4096,
        this,
        2,
        &_telemetryTaskHandle,
        0
    );

    return true;
}

void SlimprotoClient::stop() {
    if (!_running) return;
    _running = false;

    slimproto_stop();
    output_close();
    decode_close();
    stream_close();

    if (_telemetryTaskHandle) {
        vTaskDelay(pdMS_TO_TICKS(50));
        _telemetryTaskHandle = nullptr;
    }
    Serial.println("[SLIMPROTO] Squeezelite pipeline stopped");
}

void SlimprotoClient::_telemetryTaskTrampoline(void* arg) {
    static_cast<SlimprotoClient*>(arg)->_runTelemetry();
}

void SlimprotoClient::_runTelemetry() {
    uint32_t lastPrintMs = 0;
    uint32_t idleSinceMs = 0;

    while (_running) {
        vTaskDelay(pdMS_TO_TICKS(250));

        WiFiStreamMetadata meta = _params.getWiFiStreamMetadata();

        bool isStream = (output.state == OUTPUT_RUNNING || output.state == OUTPUT_BUFFER);
        bool isPaused = (output.state == OUTPUT_STOPPED || output.state == OUTPUT_PAUSE_FRAMES);

        meta.connected  = _running;
        meta.streaming  = isStream;
        meta.paused     = isPaused;
        meta.sampleRate = 44100;
        meta.bitDepth   = 24;
        meta.pos        = output.frames_played / 44100;

        if (isStream) {
            idleSinceMs = 0;
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "PCM 44.1kHz 24-bit");
            if (_params.getInputMode() != INPUT_MODE_WIFI) {
                Serial.println("[SLIMPROTO] Auto-switching input mode to Squeezebox");
                _params.setInputMode(INPUT_MODE_WIFI);
            }
        } else {
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "Idle");
            if (!isPaused && _params.getInputMode() == INPUT_MODE_WIFI && output.state == OUTPUT_OFF) {
                if (idleSinceMs == 0) idleSinceMs = millis();
                if (millis() - idleSinceMs >= 3000) {
                    _params.setInputMode(INPUT_MODE_AUX);
                    idleSinceMs = 0;
                }
            } else {
                idleSinceMs = 0;
            }
        }

        _params.setWiFiStreamMetadata(meta);

        uint32_t now = millis();
        if (isStream && (now - lastPrintMs >= 5000)) {
            lastPrintMs = now;
            wifi_ps_type_t ps = WIFI_PS_NONE;
            esp_wifi_get_ps(&ps);
            Serial.printf("[SLIMPROTO STATS] state=%d | stream_buf=%u KB | out_buf=%u KB (%u frames) | played=%u s | rssi=%d dBm | ps=%d\n",
                          output.state,
                          (unsigned)(_buf_used(streambuf) / 1024),
                          (unsigned)(_buf_used(outputbuf) / 1024),
                          (unsigned)(_buf_used(outputbuf) / BYTES_PER_FRAME),
                          (unsigned)meta.pos,
                          (int)WiFi.RSSI(),
                          (int)ps);
        }
    }

    vTaskDelete(NULL);
}
