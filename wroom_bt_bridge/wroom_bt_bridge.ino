#include <Arduino.h>
#include <Preferences.h>
#include "BluetoothA2DPSink.h"

#define I2S_BCK_PIN   22
#define I2S_WS_PIN    21
#define I2S_DOUT_PIN  23
#define LED_PIN        2

#define SAMPLE_RATE   44100
#define BT_DEV_NAME   "ESP32-DSP-BT"

BluetoothA2DPSink a2dp_sink;

static Preferences g_prefs;
static uint8_t g_btVolume = 100;
static volatile bool g_volDirty = false;
static volatile unsigned long g_lastVolChangeTime = 0;
static const unsigned long kVolSaveDebounceMs = 2500;

enum BtState {
    BT_DISCONNECTED,
    BT_CONNECTED_IDLE,
    BT_PLAYING
};
static volatile BtState g_btState = BT_DISCONNECTED;

void save_volume_to_nvs() {
    if (!g_volDirty) return;
    g_volDirty = false;
    uint8_t vol = g_btVolume;
    if (g_prefs.begin("bt_cfg", false)) {
        g_prefs.putUChar("vol", vol);
        g_prefs.end();
        Serial.printf("[NVS] Persisted Bluetooth volume %d / 127 (%d%%) to flash.\n",
                      vol, (uint32_t)vol * 100 / 127);
    }
}

void avrc_volume_change_callback(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 127) vol = 127;
    if (static_cast<uint8_t>(vol) != g_btVolume) {
        g_btVolume = static_cast<uint8_t>(vol);
        g_lastVolChangeTime = millis();
        g_volDirty = true;
        Serial.printf("[AVRC] Volume updated by host: %d / 127 (%d%%)\n",
                      g_btVolume, (uint32_t)g_btVolume * 100 / 127);
    }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    switch (state) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
            Serial.println("[BT] Device disconnected. Ready for pairing...");
            if (g_volDirty) {
                g_lastVolChangeTime = 0;
            }
            g_btState = BT_DISCONNECTED;
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            Serial.println("[BT] Connecting to Bluetooth source...");
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            Serial.println("[BT] Connected to Bluetooth source!");
            g_btState = BT_CONNECTED_IDLE;
            a2dp_sink.set_volume(g_btVolume);
            Serial.printf("[BT] Enforced remembered volume: %d / 127 (%d%%)\n",
                          g_btVolume, (uint32_t)g_btVolume * 100 / 127);
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            Serial.println("[BT] Disconnecting...");
            break;
    }
}

void audio_state_changed(esp_a2d_audio_state_t state, void *ptr) {
    switch (state) {
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        case ESP_A2D_AUDIO_STATE_STOPPED:
            Serial.println("[BT] Audio stopped/paused");
            if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
            break;
        case ESP_A2D_AUDIO_STATE_STARTED:
            Serial.println("[BT] Audio streaming active -> transmitting I2S to S3");
            g_btState = BT_PLAYING;
            break;
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(1000);
    Serial.println("\n==================================================");
    Serial.println("  ESP32 DevKit V1 Bluetooth A2DP Bridge           ");
    Serial.println("==================================================");
    Serial.printf("[INIT] Output Sample Rate : %d Hz\n", SAMPLE_RATE);
    Serial.printf("[INIT] I2S Pins           : BCK=%d, WS=%d, DOUT=%d\n",
                  I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
    Serial.printf("[INIT] Bluetooth Device   : '%s'\n", BT_DEV_NAME);

    i2s_pin_config_t pin_config = {
        .mck_io_num   = I2S_PIN_NO_CHANGE,
        .bck_io_num   = I2S_BCK_PIN,
        .ws_io_num    = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    i2s_config_t i2s_config = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = 0,
        .dma_buf_count        = 8,
        .dma_buf_len          = 256,
        .use_apll             = true,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };

    a2dp_sink.set_pin_config(pin_config);
    a2dp_sink.set_i2s_config(i2s_config);

    g_prefs.begin("bt_cfg", false);
    g_btVolume = g_prefs.getUChar("vol", 100);
    g_prefs.end();
    Serial.printf("[INIT] Loaded Bluetooth volume from NVS: %d / 127 (%d%%)\n",
                  g_btVolume, (uint32_t)g_btVolume * 100 / 127);

    a2dp_sink.set_on_connection_state_changed(connection_state_changed);
    a2dp_sink.set_on_audio_state_changed(audio_state_changed);
    a2dp_sink.set_on_volumechange(avrc_volume_change_callback);

    a2dp_sink.start(BT_DEV_NAME);
    a2dp_sink.set_volume(g_btVolume);
    Serial.printf("[INIT] Bluetooth A2DP Sink started. Initial volume: %d / 127.\n", g_btVolume);
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastLedUpdate = 0;
    static bool ledState = false;

    switch (g_btState) {
        case BT_DISCONNECTED:
            if (now - lastLedUpdate >= 250) {
                lastLedUpdate = now;
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            }
            break;

        case BT_CONNECTED_IDLE:
            if (now - lastLedUpdate >= 1000) {
                lastLedUpdate = now;
                ledState = !ledState;
                digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            }
            break;

        case BT_PLAYING:
            if (!ledState) {
                ledState = true;
                digitalWrite(LED_PIN, HIGH);
            }
            break;
    }

    if (g_volDirty && (now - g_lastVolChangeTime >= kVolSaveDebounceMs)) {
        save_volume_to_nvs();
    }

    delay(10);
}
