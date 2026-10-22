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
static volatile unsigned long g_lastVolChangeTime = 0;
static const unsigned long kVolSaveDebounceMs = 2500;

enum BtState {
    BT_DISCONNECTED,
    BT_CONNECTED_IDLE,
    BT_PLAYING
};
static volatile BtState g_btState = BT_DISCONNECTED;

static portMUX_TYPE g_metaMux = portMUX_INITIALIZER_UNLOCKED;
static char g_trackTitle[128]  = "No Track Playing";
static char g_trackArtist[128] = "Unknown Artist";
static char g_trackAlbum[128]  = "Unknown Album";
static uint32_t g_trackDurationSec = 0;
static uint32_t g_trackPosSec = 0;
static volatile bool g_metadataChanged = false;
static volatile bool g_trackInfoChanged = false;
static volatile unsigned long g_lastMetaUpdateMs = 0;
static unsigned long g_lastMetaBroadcast = 0;

void broadcast_metadata() {
    char title[128];
    char artist[128];
    char album[128];
    uint32_t durationSec;
    uint32_t posSec;
    BtState state;
    bool logBroadcast;

    portENTER_CRITICAL(&g_metaMux);
    memcpy(title,  g_trackTitle,  sizeof(title));
    memcpy(artist, g_trackArtist, sizeof(artist));
    memcpy(album,  g_trackAlbum,  sizeof(album));
    durationSec  = g_trackDurationSec;
    posSec       = g_trackPosSec;
    state        = g_btState;
    logBroadcast = g_trackInfoChanged;
    g_metadataChanged  = false;
    g_trackInfoChanged = false;
    portEXIT_CRITICAL(&g_metaMux);

    StaticJsonDocument<384> doc;
    doc["t"]         = "bt_meta";
    doc["title"]     = title;
    doc["artist"]    = artist;
    doc["album"]     = album;
    doc["connected"] = (state != BT_DISCONNECTED);
    doc["playing"]   = (state == BT_PLAYING);
    doc["duration"]  = durationSec;
    doc["pos"]       = posSec;

    char line[384];
    size_t len = serializeJson(doc, line, sizeof(line) - 2);
    line[len] = '\n';
    line[len + 1] = '\0';
    S3Serial.write(reinterpret_cast<const uint8_t*>(line), len + 1);

    if (logBroadcast) {
        line[len] = '\0';
        Serial.printf("[META] Broadcasted -> S3: %s\n", line);
    }
    g_lastMetaBroadcast = millis();
}

void avrc_play_pos_callback(uint32_t play_pos) {
    uint32_t sec = play_pos / 1000;
    portENTER_CRITICAL(&g_metaMux);
    if (sec != g_trackPosSec) {
        g_trackPosSec = sec;
        g_metadataChanged = true;
    }
    portEXIT_CRITICAL(&g_metaMux);
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
    if (!text) return;
    const char* str = reinterpret_cast<const char*>(text);
    Serial.printf("[AVRC] Attr 0x%02X: %s\n", id, str);

    portENTER_CRITICAL(&g_metaMux);
    if (id == ESP_AVRC_MD_ATTR_TITLE) {
        strncpy(g_trackTitle, str, sizeof(g_trackTitle) - 1);
        g_trackTitle[sizeof(g_trackTitle) - 1] = '\0';
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        g_lastMetaUpdateMs = millis();
    } else if (id == ESP_AVRC_MD_ATTR_ARTIST) {
        strncpy(g_trackArtist, str, sizeof(g_trackArtist) - 1);
        g_trackArtist[sizeof(g_trackArtist) - 1] = '\0';
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        g_lastMetaUpdateMs = millis();
    } else if (id == ESP_AVRC_MD_ATTR_ALBUM) {
        strncpy(g_trackAlbum, str, sizeof(g_trackAlbum) - 1);
        g_trackAlbum[sizeof(g_trackAlbum) - 1] = '\0';
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        g_lastMetaUpdateMs = millis();
    } else if (id == ESP_AVRC_MD_ATTR_PLAYING_TIME) {
        uint32_t ms = strtoul(str, nullptr, 10);
        g_trackDurationSec = ms / 1000;
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        g_lastMetaUpdateMs = millis();
    }
    portEXIT_CRITICAL(&g_metaMux);
}

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
        Serial.printf("[AVRC] Volume updated by host: %d / 127 (%d%%) [pending NVS save]\n",
                      g_btVolume, (uint32_t)g_btVolume * 100 / 127);
    }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    switch (state) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
            Serial.println("[BT] Device disconnected. Ready for pairing...");
            if (g_volDirty) {
                g_lastVolChangeTime = 0; // trigger immediate NVS save on loopTask
            }
            portENTER_CRITICAL(&g_metaMux);
            g_btState = BT_DISCONNECTED;
            strncpy(g_trackTitle, "No Track Playing", sizeof(g_trackTitle) - 1);
            strncpy(g_trackArtist, "Unknown Artist", sizeof(g_trackArtist) - 1);
            strncpy(g_trackAlbum, "Unknown Album", sizeof(g_trackAlbum) - 1);
            g_trackDurationSec = 0;
            g_trackPosSec = 0;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            Serial.println("[BT] Connecting to Bluetooth source...");
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            Serial.println("[BT] Connected to Bluetooth source!");
            portENTER_CRITICAL(&g_metaMux);
            g_btState = BT_CONNECTED_IDLE;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
            a2dp_sink.set_volume(g_btVolume); // Enforce remembered volume on connection
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
            portENTER_CRITICAL(&g_metaMux);
            if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
            break;
        case ESP_A2D_AUDIO_STATE_STARTED:
            Serial.println("[BT] Audio streaming active -> transmitting I2S to S3");
            portENTER_CRITICAL(&g_metaMux);
            g_btState = BT_PLAYING;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
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

    g_trackInfoChanged = true;
    broadcast_metadata();
}

void handle_s3_command(const char* cmd) {
    Serial.printf("[AVRC-CMD] Received from S3: '%s'\n", cmd);
    if (strcasecmp(cmd, "play") == 0) {
        a2dp_sink.play();
        portENTER_CRITICAL(&g_metaMux);
        g_btState = BT_PLAYING;
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        portEXIT_CRITICAL(&g_metaMux);
        Serial.println("[AVRC-CMD] Executed -> PLAY");
    } else if (strcasecmp(cmd, "pause") == 0) {
        a2dp_sink.pause();
        portENTER_CRITICAL(&g_metaMux);
        if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
        g_metadataChanged = true;
        g_trackInfoChanged = true;
        portEXIT_CRITICAL(&g_metaMux);
        Serial.println("[AVRC-CMD] Executed -> PAUSE");
    } else if (strcasecmp(cmd, "next") == 0) {
        a2dp_sink.next();
        Serial.println("[AVRC-CMD] Executed -> NEXT");
    } else if (strcasecmp(cmd, "prev") == 0 || strcasecmp(cmd, "previous") == 0) {
        a2dp_sink.previous();
        Serial.println("[AVRC-CMD] Executed -> PREVIOUS");
    } else if (strcasecmp(cmd, "play_pause") == 0) {
        if (g_btState == BT_PLAYING) {
            a2dp_sink.pause();
            portENTER_CRITICAL(&g_metaMux);
            if (g_btState != BT_DISCONNECTED) g_btState = BT_CONNECTED_IDLE;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
            Serial.println("[AVRC-CMD] Executed -> PAUSE (toggle)");
        } else {
            a2dp_sink.play();
            portENTER_CRITICAL(&g_metaMux);
            g_btState = BT_PLAYING;
            g_metadataChanged = true;
            g_trackInfoChanged = true;
            portEXIT_CRITICAL(&g_metaMux);
            Serial.println("[AVRC-CMD] Executed -> PLAY (toggle)");
        }
    }
    broadcast_metadata();
}

void loop() {
    static char rxLine[64];
    static size_t rxLen = 0;

    while (S3Serial.available()) {
        char c = static_cast<char>(S3Serial.read());
        if (c == '\n') {
            while (rxLen > 0 && (rxLine[rxLen - 1] == ' ' || rxLine[rxLen - 1] == '\t')) {
                rxLen--;
            }
            rxLine[rxLen] = '\0';
            char* start = rxLine;
            while (*start == ' ' || *start == '\t') start++;
            if (*start != '\0') {
                handle_s3_command(start);
            }
            rxLen = 0;
        } else if (c != '\r' && rxLen < sizeof(rxLine) - 1) {
            rxLine[rxLen++] = c;
        }
    }

    unsigned long now = millis();
    if (g_metadataChanged && (now - g_lastMetaBroadcast >= 100) && (now - g_lastMetaUpdateMs >= 40)) {
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
