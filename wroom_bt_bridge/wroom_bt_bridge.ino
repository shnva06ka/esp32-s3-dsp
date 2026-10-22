// Bluetooth A2DP sink for the ESP32-WROOM. Audio goes to the S3 over I2S (S3 is the slave), track info goes as JSON over UART.
// Wiring is in the README.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "BluetoothA2DPSink.h"

#define I2S_BCK_PIN    22   // Bit Clock to ESP32-S3 GPIO 16
#define I2S_WS_PIN     21   // Word Select to ESP32-S3 GPIO 17
#define I2S_DOUT_PIN   23   // Audio Data Out to ESP32-S3 GPIO 18
#define S3_UART_TX_PIN 19   // UART TX to ESP32-S3 GPIO 44 (RX)
#define S3_UART_RX_PIN 18   // UART RX from ESP32-S3 GPIO 43 (TX)
#define LED_PIN         2   // Standard built-in blue LED on DevKit V1

#define SAMPLE_RATE   48000 // has to match the S3
#define BT_DEV_NAME   "ESP32-DSP-BT"

BluetoothA2DPSink a2dp_sink;
HardwareSerial S3Serial(2); // UART2 (Full-duplex to S3)

// volume goes to NVS 2.5 s after the last change (flash wear)
static Preferences g_prefs;
static uint8_t g_btVolume = 100; // Default volume ~80% (100 / 127) if not yet stored
static volatile bool g_volDirty = false;
static unsigned long g_lastVolChangeTime = 0;
static const unsigned long kVolSaveDebounceMs = 2500;

enum BtState {
    BT_DISCONNECTED,
    BT_CONNECTED_IDLE,
    BT_PLAYING
};
static volatile BtState g_btState = BT_DISCONNECTED;

static char g_trackTitle[128]  = "No Track Playing";
static char g_trackArtist[128] = "Unknown Artist";
static char g_trackAlbum[128]  = "Unknown Album";
static uint32_t g_trackDurationMs = 0;
static uint32_t g_trackPosMs = 0;
static volatile bool g_metadataChanged = false;
static unsigned long g_lastMetaBroadcast = 0;

void broadcast_metadata() {
    StaticJsonDocument<384> doc;
    doc["t"]         = "bt_meta";
    doc["title"]     = g_trackTitle;
    doc["artist"]    = g_trackArtist;
    doc["album"]     = g_trackAlbum;
    doc["connected"] = (g_btState != BT_DISCONNECTED);
    doc["playing"]   = (g_btState == BT_PLAYING);
    doc["duration"]  = g_trackDurationMs / 1000; // in seconds
    doc["pos"]       = g_trackPosMs / 1000;      // in seconds

    String line;
    serializeJson(doc, line);
    S3Serial.println(line);
    Serial.printf("[META] Broadcasted -> S3: %s\n", line.c_str());
    g_metadataChanged = false;
    g_lastMetaBroadcast = millis();
}

void avrc_play_pos_callback(uint32_t play_pos) {
    g_trackPosMs = play_pos;
    g_metadataChanged = true;
    Serial.printf("[AVRC] Play pos: %u ms\n", play_pos);
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
    if (!text) return;
    const char* str = reinterpret_cast<const char*>(text);
    Serial.printf("[AVRC] Attr 0x%02X: %s\n", id, str);

    if (id == ESP_AVRC_MD_ATTR_TITLE) {
        strncpy(g_trackTitle, str, sizeof(g_trackTitle) - 1);
        g_trackTitle[sizeof(g_trackTitle) - 1] = '\0';
        g_metadataChanged = true;
    } else if (id == ESP_AVRC_MD_ATTR_ARTIST) {
        strncpy(g_trackArtist, str, sizeof(g_trackArtist) - 1);
        g_trackArtist[sizeof(g_trackArtist) - 1] = '\0';
        g_metadataChanged = true;
    } else if (id == ESP_AVRC_MD_ATTR_ALBUM) {
        strncpy(g_trackAlbum, str, sizeof(g_trackAlbum) - 1);
        g_trackAlbum[sizeof(g_trackAlbum) - 1] = '\0';
        g_metadataChanged = true;
    } else if (id == ESP_AVRC_MD_ATTR_PLAYING_TIME) {
        g_trackDurationMs = strtoul(str, nullptr, 10);
        g_metadataChanged = true;
        Serial.printf("[AVRC] Track duration: %u ms\n", g_trackDurationMs);
    }
}

void save_volume_to_nvs() {
    if (!g_volDirty) return;
    g_volDirty = false;
    if (g_prefs.begin("bt_cfg", false)) {
        g_prefs.putUChar("vol", g_btVolume);
        g_prefs.end();
        Serial.printf("[NVS] Persisted Bluetooth volume %d / 127 (%d%%) to flash.\n",
                      g_btVolume, (uint32_t)g_btVolume * 100 / 127);
    }
}

void avrc_volume_change_callback(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 127) vol = 127;
    if (static_cast<uint8_t>(vol) != g_btVolume) {
        g_btVolume = static_cast<uint8_t>(vol);
        g_volDirty = true;
        g_lastVolChangeTime = millis();
        Serial.printf("[AVRC] Volume updated by host: %d / 127 (%d%%) [pending NVS save]\n",
                      g_btVolume, (uint32_t)g_btVolume * 100 / 127);
    }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    switch (state) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
            Serial.println("[BT] Device disconnected. Ready for pairing...");
            save_volume_to_nvs(); // Immediately commit pending volume to flash on disconnect
            g_btState = BT_DISCONNECTED;
            strncpy(g_trackTitle, "No Track Playing", sizeof(g_trackTitle) - 1);
            strncpy(g_trackArtist, "Unknown Artist", sizeof(g_trackArtist) - 1);
            strncpy(g_trackAlbum, "Unknown Album", sizeof(g_trackAlbum) - 1);
            g_metadataChanged = true;
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            Serial.println("[BT] Connecting to Bluetooth source...");
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            Serial.println("[BT] Connected to Bluetooth source!");
            g_btState = BT_CONNECTED_IDLE;
            a2dp_sink.set_volume(g_btVolume); // Enforce remembered volume on connection
            Serial.printf("[BT] Enforced remembered volume: %d / 127 (%d%%)\n",
                          g_btVolume, (uint32_t)g_btVolume * 100 / 127);
            g_metadataChanged = true;
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
            g_metadataChanged = true;
            break;
        case ESP_A2D_AUDIO_STATE_STARTED:
            Serial.println("[BT] Audio streaming active -> transmitting I2S to S3");
            g_btState = BT_PLAYING;
            g_metadataChanged = true;
            break;
    }
}

void setup() {
    Serial.begin(115200);
    // UART2 to S3: RX on GPIO 18 (from S3 TX GPIO 43), TX on GPIO 19 (to S3 RX GPIO 44)
    pinMode(S3_UART_RX_PIN, INPUT_PULLUP);
    S3Serial.begin(115200, SERIAL_8N1, S3_UART_RX_PIN, S3_UART_TX_PIN);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    delay(1000);
    Serial.println("\n==================================================");
    Serial.println("  ESP32 DevKit V1 Bluetooth A2DP + AVRCP Bridge  ");
    Serial.println("==================================================");
    Serial.printf("[INIT] Output Sample Rate : %d Hz\n", SAMPLE_RATE);
    Serial.printf("[INIT] I2S Pins           : BCK=%d, WS=%d, DOUT=%d\n",
                  I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
    Serial.printf("[INIT] S3 UART TX Pin     : GPIO %d -> S3 GPIO 44 (RX)\n", S3_UART_TX_PIN);
    Serial.printf("[INIT] Bluetooth Device   : '%s'\n", BT_DEV_NAME);

    i2s_pin_config_t pin_config = {
        .mck_io_num   = I2S_PIN_NO_CHANGE, // No MCLK needed for slave S3
        .bck_io_num   = I2S_BCK_PIN,
        .ws_io_num    = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    // 32-bit slots to match the S3
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

    a2dp_sink.set_avrc_metadata_attribute_mask(
        ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
        ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME
    );
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_sink.set_avrc_rn_play_pos_callback(avrc_play_pos_callback, 1); // 1-second position notifications
    a2dp_sink.set_on_connection_state_changed(connection_state_changed);
    a2dp_sink.set_on_audio_state_changed(audio_state_changed);
    a2dp_sink.set_on_volumechange(avrc_volume_change_callback);

    a2dp_sink.start(BT_DEV_NAME);
    a2dp_sink.set_volume(g_btVolume);
    Serial.printf("[INIT] Bluetooth A2DP Sink started successfully. Initial volume: %d / 127.\n", g_btVolume);
    Serial.println("[INIT] Connect your phone/PC to 'ESP32-DSP-BT'.");

    broadcast_metadata();
}

void handle_s3_command(const String& cmd) {
    Serial.printf("[AVRC-CMD] Received from S3: '%s'\n", cmd.c_str());
    if (cmd.equalsIgnoreCase("play")) {
        a2dp_sink.play();
        g_btState = BT_PLAYING;
        g_metadataChanged = true;
        Serial.println("[AVRC-CMD] Executed -> PLAY");
    } else if (cmd.equalsIgnoreCase("pause")) {
        a2dp_sink.pause();
        if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
        g_metadataChanged = true;
        Serial.println("[AVRC-CMD] Executed -> PAUSE");
    } else if (cmd.equalsIgnoreCase("next")) {
        a2dp_sink.next();
        Serial.println("[AVRC-CMD] Executed -> NEXT");
    } else if (cmd.equalsIgnoreCase("prev") || cmd.equalsIgnoreCase("previous")) {
        a2dp_sink.previous();
        Serial.println("[AVRC-CMD] Executed -> PREVIOUS");
    } else if (cmd.equalsIgnoreCase("play_pause")) {
        if (g_btState == BT_PLAYING) {
            a2dp_sink.pause();
            if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
            Serial.println("[AVRC-CMD] Executed -> PAUSE (toggle)");
        } else {
            a2dp_sink.play();
            g_btState = BT_PLAYING;
            Serial.println("[AVRC-CMD] Executed -> PLAY (toggle)");
        }
        g_metadataChanged = true;
    }
    broadcast_metadata();
}

void loop() {
    static String rxLine = "";
    while (S3Serial.available()) {
        char c = static_cast<char>(S3Serial.read());
        if (c == '\n') {
            rxLine.trim();
            if (rxLine.length() > 0) {
                handle_s3_command(rxLine);
            }
            rxLine = "";
        } else if (c != '\r' && rxLine.length() < 64) {
            rxLine += c;
        }
    }

    unsigned long now = millis();
    if (g_metadataChanged && (now - g_lastMetaBroadcast >= 100)) {
        broadcast_metadata();
    }

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
            digitalWrite(LED_PIN, HIGH); // Solid ON when streaming audio
            break;
    }

    if (g_volDirty && (now - g_lastVolChangeTime >= kVolSaveDebounceMs)) {
        save_volume_to_nvs();
    }

    delay(10);
}
