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
#include <esp_task_wdt.h>

static void audioTask(void* pvParameters);
static void webTask(void* pvParameters);

ParamStore g_params;
I2SAudio   g_audio(g_params);

static TaskHandle_t s_audioTaskHandle = nullptr;
static TaskHandle_t s_webTaskHandle   = nullptr;
static std::atomic<bool> s_audioReady{false};

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
    Console.printf("[BOOT] Flash: %u MB (Mode: DIO)\n", ESP.getFlashChipSize() / (1024 * 1024));

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

    g_params.begin();
    output_init(lINFO, OUTPUTBUF_SIZE, NULL);

    Console.println("[RESAMPLE] Running quick arbitrary-rate resampler self-test...");
    resampler_run_self_test();

    esp_task_wdt_init(10, true);

    xTaskCreatePinnedToCore(
        webTask,
        "WebTask",
        10240,
        nullptr,
        2,
        &s_webTaskHandle,
        0
    );

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        5120,
        nullptr,
        configMAX_PRIORITIES - 1,
        &s_audioTaskHandle,
        1
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
        ResamplerTelemetry::setAudioTaskHandle(nullptr);
        vTaskDelete(nullptr);
    }
    ResamplerTelemetry::setBtTaskHandle(g_audio.getBtTaskHandle());
    s_audioReady.store(true, std::memory_order_release);

    Console.println("[AUDIO] I2S_NUM_0 Master (PCM1808 ADC + PCM5102A DAC) ready");
    Console.println("[AUDIO] I2S_NUM_1 Slave (Bluetooth A2DP Receiver) ready");

    static DSPPipeline dsp(g_params);
    Console.println("[AUDIO] DSP Pipeline active on Core 1");

    esp_task_wdt_add(nullptr);

    for (;;) {
        g_audio.process([&](float* left, float* right, size_t frames) {
            dsp.process(left, right, frames);
        });
        esp_task_wdt_reset();
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

    esp_task_wdt_add(nullptr);

    bool otaConfirmed = false;
    uint32_t bootStartMs = millis();

    for (;;) {
        ui.loop();
        ResamplerTelemetry::poll();

        if (!otaConfirmed && s_audioReady.load(std::memory_order_acquire) &&
            (uint32_t)(millis() - bootStartMs) >= 5000U) {
            esp_ota_mark_app_valid_cancel_rollback();
            otaConfirmed = true;
            Console.println("[OTA] Firmware verified healthy - rollback cancelled.");
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
