// ESP32-S3-DSP entry point. Core 1 runs audio + DSP, core 0 runs WebUI, Wi-Fi and SlimProto.

#include <Arduino.h>
#include "audio_config.h"
#include "i2s_audio.h"
#include "dsp_pipeline.h"
#include "web_ui.h"
#include "sys_monitor.h"
#include "slimproto_client.h"
#include "squeezelite.h"
#include "resampler_telemetry.h"
#include "wireless_console.h"
#include <esp_ota_ops.h>

static void audioTask(void* pvParameters);
static void webTask(void* pvParameters);

ParamStore g_params;
I2SAudio   g_audio(g_params);

static TaskHandle_t s_audioTaskHandle = nullptr;
static TaskHandle_t s_webTaskHandle   = nullptr;

void setup()
{
    Serial.begin(115200);

    // Give USB CDC time to enumerate if host is connected
    delay(1200);

    Console.println("\n========================================================");
    Console.println("  ESP32-S3-DSP  -  Real-Time DSP Audio Processor");
    Console.println("========================================================");
    Console.printf("[BOOT] Chip: ESP32-S3 (Rev %d, %d Cores @ %d MHz)\n",
                  ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
    Console.printf("[BOOT] Flash: %u MB (Mode: QIO)\n", ESP.getFlashChipSize() / (1024 * 1024));

    if (psramFound()) {
        Console.printf("[BOOT] PSRAM: %u MB Octal PSRAM (OPI) Active\n",
                      ESP.getPsramSize() / (1024 * 1024));
        Console.printf("[BOOT] Free PSRAM: %u bytes\n", ESP.getFreePsram());
    } else {
        Console.println("[BOOT] WARNING: PSRAM not initialized (running on internal DRAM)");
    }
    Console.printf("[BOOT] Free Internal DRAM: %u bytes\n", ESP.getFreeHeap());
    Console.printf("[BOOT] Reset Reason: %s\n", ResamplerTelemetry::getResetReasonString());

    SystemMonitor::begin();

    // Confirm healthy boot to OTA subsystem to cancel automatic rollback
    esp_ota_mark_app_valid_cancel_rollback();

    g_params.begin();
    output_init(lINFO, OUTPUTBUF_SIZE, NULL);

    Console.println("[RESAMPLE] Running quick arbitrary-rate resampler self-test...");
    resampler_run_self_test();

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        5120,
        nullptr,
        configMAX_PRIORITIES - 1,
        &s_audioTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        10240,
        nullptr,
        2,
        &s_webTaskHandle,
        0
    );

    ResamplerTelemetry::setTaskHandles(g_audio.getBtTaskHandle(), s_audioTaskHandle, s_webTaskHandle, sl_decode_task_handle());
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

static void audioTask(void* /*pvParameters*/)
{
    Console.println("[AUDIO] Initialising Dual-I2S Audio Driver...");
    if (!g_audio.begin()) {
        Console.println("[AUDIO] FATAL: I2S peripheral initialisation failed");
        vTaskDelete(nullptr);
    }
    ResamplerTelemetry::setBtTaskHandle(g_audio.getBtTaskHandle());

    Console.println("[AUDIO] I2S_NUM_0 Master (PCM1808 ADC + PCM5102A DAC) ready");
    Console.println("[AUDIO] I2S_NUM_1 Slave (Bluetooth A2DP Receiver) ready");

    static DSPPipeline dsp(g_params);
    Console.println("[AUDIO] DSP Pipeline active on Core 1");

    for (;;) {
        g_audio.process([&](float* left, float* right, size_t frames) {
            dsp.process(left, right, frames);
        });
    }
}

static void webTask(void* /*pvParameters*/)
{
    Console.println("[WEB] Initialising Web UI on Core 0...");
    static WebUI ui(g_params);
    ui.begin();

    Console.println("[NET] Starting Squeezelite / SlimProto Wi-Fi streaming client on Core 0...");
    static SlimprotoClient slimproto(g_params);
    slimproto.begin();

    for (;;) {
        ui.loop();
        ResamplerTelemetry::poll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
