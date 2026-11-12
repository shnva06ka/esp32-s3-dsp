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
extern "C" volatile unsigned sl_source_rate;
extern "C" volatile unsigned sl_source_bits;

SlimprotoClient::SlimprotoClient(ParamStore& params)
    : _params(params)
{
}

SlimprotoClient::~SlimprotoClient() {
    stop();
}

bool SlimprotoClient::begin() {
    if (_running.load(std::memory_order_acquire)) return true;

    Console.println("[SLIMPROTO] Starting Squeezelite streaming pipeline...");

    uint8_t mac[6];
    WiFi.macAddress(mac);

    output_init(lINFO, OUTPUTBUF_SIZE, NULL);
    stream_init(lINFO, STREAMBUF_SIZE);
    decode_init(lINFO, "pcm,flc", "");

    slimproto_start(lINFO, NULL, mac, "ESP32-S3-DSP", SAMPLE_RATE);

    _running.store(true, std::memory_order_release);
    _telemetryExited.store(false, std::memory_order_release);
    if (xTaskCreatePinnedToCore(
            _telemetryTaskTrampoline,
            "sl_telemetry",
            4096,
            this,
            2,
            &_telemetryTaskHandle,
            0
        ) != pdPASS) {
        _telemetryTaskHandle = nullptr;
        _telemetryExited.store(true, std::memory_order_release);
    }

    return true;
}

void SlimprotoClient::stop() {
    if (!_running.exchange(false, std::memory_order_acq_rel)) return;

    slimproto_stop();
    if (_telemetryTaskHandle) {
        for (int w = 0; w < 60 && !_telemetryExited.load(std::memory_order_acquire); ++w) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        _telemetryTaskHandle = nullptr;
    }
    decode_close();
    stream_close();
    output_close();

    Console.println("[SLIMPROTO] Squeezelite pipeline stopped");
}

void SlimprotoClient::_telemetryTaskTrampoline(void* arg) {
    static_cast<SlimprotoClient*>(arg)->_runTelemetry();
}

void SlimprotoClient::_runTelemetry() {
    uint32_t lastPrintMs = 0;
    uint32_t idleSinceMs = 0;

    while (_running.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(250));

        WiFiStreamMetadata meta = _params.getWiFiStreamMetadata();

        output_state curOutState = OUTPUT_STOPPED;
        u32_t curFramesPlayed = 0;
        unsigned outUsedBytes = 0;
        if (outputbuf && outputbuf->mutex) {
            mutex_lock(outputbuf->mutex);
            curOutState = output.state;
            curFramesPlayed = output.frames_played;
            outUsedBytes = _buf_used(outputbuf);
            mutex_unlock(outputbuf->mutex);
        }

        unsigned streamUsedBytes = 0;
        if (streambuf && streambuf->mutex) {
            mutex_lock(streambuf->mutex);
            streamUsedBytes = _buf_used(streambuf);
            mutex_unlock(streambuf->mutex);
        }

        bool isStream = (curOutState == OUTPUT_RUNNING || curOutState == OUTPUT_BUFFER);
        bool isPaused = (curOutState == OUTPUT_STOPPED || curOutState == OUTPUT_PAUSE_FRAMES);

        meta.connected  = slimproto_is_connected();
        slimproto_get_server_ip(meta.serverIp, sizeof(meta.serverIp));
        if (meta.connected && meta.serverIp[0]) {
            snprintf(meta.serverName, sizeof(meta.serverName), "SlimProto (%s)", meta.serverIp);
        } else {
            snprintf(meta.serverName, sizeof(meta.serverName), "None");
        }
        meta.streaming  = isStream;
        meta.paused     = isPaused;
        meta.sampleRate = sl_source_rate ? sl_source_rate : (uint32_t)SAMPLE_RATE;
        meta.bitDepth   = sl_source_bits ? sl_source_bits : 16;
        meta.pos        = curFramesPlayed / (uint32_t)SAMPLE_RATE;

        if (isStream) {
            idleSinceMs = 0;
            unsigned srcRate = sl_source_rate ? sl_source_rate : (unsigned)SAMPLE_RATE;
            unsigned bits = sl_source_bits ? sl_source_bits : 16;
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "%u.%u kHz %u-bit",
                     srcRate / 1000, (srcRate % 1000) / 100, bits);
            if (_params.getInputMode() != INPUT_MODE_WIFI) {
                Console.println("[SLIMPROTO] Auto-switching input mode to Squeezebox");
                _params.setInputMode(INPUT_MODE_WIFI, false);
            }
        } else {
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "Idle");
            if (!isPaused && _params.getInputMode() == INPUT_MODE_WIFI && curOutState == OUTPUT_OFF) {
                if (idleSinceMs == 0) idleSinceMs = millis();
                if (millis() - idleSinceMs >= 3000) {
                    _params.setInputMode(INPUT_MODE_AUX, false);
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
            Console.printf("[SLIMPROTO STATS] state=%d | stream_buf=%u KB | out_buf=%u KB (%u frames) | played=%u s | rssi=%d dBm | ps=%d\n",
                          curOutState,
                          (unsigned)(streamUsedBytes / 1024),
                          (unsigned)(outUsedBytes / 1024),
                          (unsigned)(outUsedBytes / BYTES_PER_FRAME),
                          (unsigned)meta.pos,
                          (int)WiFi.RSSI(),
                          (int)ps);
        }
    }

    _telemetryExited.store(true, std::memory_order_release);
    vTaskDelete(NULL);
}
