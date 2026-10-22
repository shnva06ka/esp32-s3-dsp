// ESP32-S3-DSP entry point. Core 1 runs audio + DSP, core 0 runs WebUI, Wi-Fi and SlimProto.

#include <Arduino.h>
#include "audio_config.h"
#include "i2s_audio.h"
#include "dsp_pipeline.h"
#include "web_ui.h"
#include "sys_monitor.h"
#include "slimproto_client.h"

static void audioTask(void* pvParameters);
static void webTask(void* pvParameters);

ParamStore g_params;
I2SAudio   g_audio(g_params);

void setup()
{
    Serial.begin(115200);

    // Give USB CDC time to enumerate if host is connected
    delay(1200);

    Serial.println("\n========================================================");
    Serial.println("  ESP32-S3-DSP  -  Real-Time DSP Audio Processor");
    Serial.println("========================================================");
    Serial.printf("[BOOT] Chip: ESP32-S3 (Rev %d, %d Cores @ %d MHz)\n",
                  ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("[BOOT] Flash: %u MB (Mode: QIO)\n", ESP.getFlashChipSize() / (1024 * 1024));

    if (psramFound()) {
        Serial.printf("[BOOT] PSRAM: %u MB Octal PSRAM (OPI) Active\n",
                      ESP.getPsramSize() / (1024 * 1024));
        Serial.printf("[BOOT] Free PSRAM: %u bytes\n", ESP.getFreePsram());
    } else {
        Serial.println("[BOOT] WARNING: PSRAM not initialized (running on internal DRAM)");
    }
    Serial.printf("[BOOT] Free Internal DRAM: %u bytes\n", ESP.getFreeHeap());

    SystemMonitor::begin();

    g_params.begin();

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        16384,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        16384,
        nullptr,
        2,
        nullptr,
        0
    );
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

static void audioTask(void* /*pvParameters*/)
{
    Serial.println("[AUDIO] Initialising Dual-I2S Audio Driver...");
    if (!g_audio.begin()) {
        Serial.println("[AUDIO] FATAL: I2S peripheral initialisation failed");
        vTaskDelete(nullptr);
    }
    Serial.println("[AUDIO] I2S_NUM_0 Master (PCM1808 ADC + PCM5102A DAC) ready");
    Serial.println("[AUDIO] I2S_NUM_1 Slave (Bluetooth A2DP Receiver) ready");

    static DSPPipeline dsp(g_params);
    Serial.println("[AUDIO] DSP Pipeline active on Core 1");

    for (;;) {
        g_audio.process([&](float* left, float* right, size_t frames) {
            dsp.process(left, right, frames);
        });
    }
}

static void webTask(void* /*pvParameters*/)
{
    Serial.println("[WEB] Initialising Web UI on Core 0...");
    static WebUI ui(g_params);
    ui.begin();

    Serial.println("[NET] Starting Squeezelite / SlimProto Wi-Fi streaming client on Core 0...");
    static SlimprotoClient slimproto(g_params, g_audio.getWiFiRingBuffer());
    slimproto.begin();

    for (;;) {
        ui.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
