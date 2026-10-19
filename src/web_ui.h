/**
 * @file web_ui.h
 * @brief Async Web UI server (Core 0) - PROGMEM page + WebSocket control.
 *
 * Architecture
 * ------------
 *  ESPAsyncWebServer (port 80)
 *  |-- GET /            -> send EQ_HTML from PROGMEM (no SPIFFS needed)
 *  |-- GET /api/params  -> JSON snapshot of all params (REST fallback)
 *  \-- WS  /ws          -> bidirectional real-time channel
 *
 * Thread safety
 * -------------
 *  All param reads/writes go through ParamStore's mutex.
 */
#pragma once

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "eq_html.h"        // PROGMEM EQ_HTML[]
#include "param_store.h"
#include "sys_monitor.h"

// -- Wi-Fi credentials (reads private wifi_credentials.h if present) -
#if __has_include("wifi_credentials.h")
#  include "wifi_credentials.h"
#endif

#ifndef WIFI_SSID
#  define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#  define WIFI_PASS ""
#endif

#ifndef BT_UART_RX_PIN
#  define BT_UART_RX_PIN 44
#endif
#ifndef BT_UART_TX_PIN
#  define BT_UART_TX_PIN 43
#endif

class WebUI {
public:
    explicit WebUI(ParamStore& params)
        : _params(params),
          _server(80),
          _ws("/ws")
    {}

    // -- Public API ------------------------------------------
    void begin() {
        Serial1.begin(115200, SERIAL_8N1, BT_UART_RX_PIN, BT_UART_TX_PIN);
        Serial.printf("[WEB] Inter-board Bluetooth metadata UART listener on RX GPIO %d\n", BT_UART_RX_PIN);
        _startWiFi();
        _setupRoutes();
        _server.addHandler(&_ws);
        _server.begin();
        Serial.println("[WEB] HTTP & WebSocket server started");
    }

    /**
     * @brief Call from the web task loop on Core 0.
     * Cleans up stale WS clients, checks for active input updates,
     * reads incoming metadata from ESP32-WROOM, and periodically prints IP.
     */
    void loop() {
        _ws.cleanupClients();

        // 1. Read incoming metadata lines from ESP32-WROOM via UART
        while (Serial1.available()) {
            char c = static_cast<char>(Serial1.read());
            if (c == '\n') {
                _uartLine.trim();
                if (_uartLine.length() > 0) {
                    _processUartLine(_uartLine);
                }
                _uartLine = "";
            } else if (c != '\r' && _uartLine.length() < 512) {
                _uartLine += c;
            }
        }

        // 2. Broadcast active source changes
        InputSourceMode mode;
        ActiveInputSource active;
        if (_params.getInputStateIfChanged(mode, active, _lastInputVersion)) {
            _ws.textAll(_buildStateJson());
        }

        // 3. Broadcast Bluetooth metadata updates
        BluetoothMetadata bt;
        if (_params.getBluetoothMetadataIfChanged(bt, _lastBtMetaVersion)) {
            _ws.textAll(_buildBtMetaJson(bt));
        }

        // 3b. Broadcast Wi-Fi streaming metadata updates
        WiFiStreamMetadata wifiMeta;
        if (_params.getWiFiStreamMetadataIfChanged(wifiMeta, _lastWifiMetaVersion)) {
            _ws.textAll(_buildWiFiMetaJson(wifiMeta));
        }

        // 4. Print IP address every 5 seconds during operation
        unsigned long now = millis();
        if (now - _lastIpPrint >= 5000) {
            _lastIpPrint = now;
            String ipStr = (WiFi.getMode() & WIFI_MODE_STA && WiFi.status() == WL_CONNECTED)
                             ? WiFi.localIP().toString()
                             : WiFi.softAPIP().toString();
            Serial.printf("[NET] Web UI accessible at: http://%s\n", ipStr.c_str());
        }

        // 5. Automatic debounced flash persistence
        if (_params.isDirty()) {
            if (now - _params.getLastDirtyTime() >= 1000) {
                _params.save();
            }
        }

        // 6. Broadcast real-time system stats (CPU, Flash, RAM) to connected WebSockets
        if (now - _lastSysBroadcast >= 1000) {
            _lastSysBroadcast = now;
            if (_ws.count() > 0) {
                _ws.textAll(_buildSysJson());
            }
        }
    }

private:
    ParamStore&    _params;
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    unsigned long  _lastIpPrint{0};
    unsigned long  _lastSysBroadcast{0};
    uint32_t       _lastInputVersion{1};
    uint32_t       _lastBtMetaVersion{1};
    uint32_t       _lastWifiMetaVersion{1};
    String         _uartLine{""};

    // -- Wi-Fi / AP setup ------------------------------------
    void _startWiFi() {
        bool hasCredentials = (strlen(WIFI_SSID) > 0 && strcmp(WIFI_SSID, "YourSSID") != 0);

        if (hasCredentials) {
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            Serial.printf("[WEB] Connecting to Wi-Fi SSID '%s'", WIFI_SSID);
            for (uint8_t i = 0; i < 20 && WiFi.status() != WL_CONNECTED; ++i) {
                delay(500);
                Serial.print('.');
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("\n[WEB] Wi-Fi connected! Station IP: %s\n", WiFi.localIP().toString().c_str());
                return;
            }
            Serial.println("\n[WEB] Wi-Fi connection timed out");
        }

        // Start fallback SoftAP named "ESP32-S3-DSP"
        Serial.println("[WEB] Starting fallback AP 'ESP32-S3-DSP'...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32-S3-DSP");
        Serial.printf("[WEB] Fallback AP active! Network SSID: 'ESP32-S3-DSP' | AP IP: %s\n",
                      WiFi.softAPIP().toString().c_str());
        Serial.printf("[NET] Web UI accessible at: http://%s\n", WiFi.softAPIP().toString().c_str());
    }

    // -- Route setup -----------------------------------------
    void _setupRoutes() {
        _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c,
                           AwsEventType type, void* arg, uint8_t* data, size_t len) {
            _onWsEvent(s, c, type, arg, data, len);
        });

        _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
            AsyncWebServerResponse* res = req->beginResponse(200, "text/html", PAGE_INDEX_HTML_GZ, PAGE_INDEX_HTML_GZ_LEN);
            res->addHeader("Content-Encoding", "gzip");
            res->addHeader("Cache-Control", "no-cache");
            req->send(res);
        });

        _server.on("/api/params", HTTP_GET, [this](AsyncWebServerRequest* req) {
            req->send(200, "application/json", _buildStateJson());
        });

        _server.onNotFound([](AsyncWebServerRequest* req) {
            req->send(404, "text/plain", "Not found");
        });
    }

    // -- WebSocket event handler ------------------------------
    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT:
                Serial.printf("[WS] Client #%u connected from %s\n",
                              client->id(), client->remoteIP().toString().c_str());
                client->setCloseClientOnQueueFull(false);
                client->text(_buildStateJson());
                client->text(_buildSysJson());
                client->text(_buildBtMetaJson(_params.getBluetoothMetadata()));
                break;
            case WS_EVT_DISCONNECT:
                Serial.printf("[WS] Client #%u disconnected\n", client->id());
                break;
            case WS_EVT_DATA: {
                auto* info = static_cast<AwsFrameInfo*>(arg);
                if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                    _handleWsMessage(client, reinterpret_cast<const char*>(data), len);
                }
                break;
            }
            default:
                break;
        }
    }

    // -- Inbound WS message dispatcher ------------------------
    void _handleWsMessage(AsyncWebSocketClient* client, const char* json, size_t len) {
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, json, len) != DeserializationError::Ok) return;

        const char* t = doc["t"] | "";

        if (strcmp(t, "eq") == 0 || strcmp(t, "eq_band") == 0) {
            _handleEQMsg(doc);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "eq_toggle") == 0 || strcmp(t, "eq_module") == 0) {
            if (doc.containsKey("enabled")) {
                _params.setEQEnabled(doc["enabled"].as<bool>());
            }
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "eq_preamp") == 0) {
            if (doc.containsKey("preamp")) {
                _params.setPreamp(doc["preamp"].as<float>());
            }
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "eq_preset") == 0) {
            const char* preset = doc["preset"] | doc["name"] | "flat";
            _params.applyPreset(preset);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "add_band") == 0) {
            float f = doc["freq"] | 1000.0f;
            float g = doc["gain"] | 0.0f;
            float q = doc["q"]    | 0.707f;
            uint8_t type = doc["type"] | 0;
            uint8_t order = doc["order"] | 2;
            _params.addEQBand(f, g, q, type, order);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "remove_band") == 0) {
            int b = doc["band"] | -1;
            _params.removeEQBand(b);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "subsonic") == 0) {
            _handleSubsonicMsg(doc);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "compander") == 0) {
            _handleCompanderMsg(doc);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "limiter") == 0) {
            _handleLimiterMsg(doc);
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "input_source") == 0 || strcmp(t, "input") == 0) {
            if (doc.containsKey("source")) {
                uint8_t src = doc["source"].as<uint8_t>();
                if (src == 1 || src == 2 || src == 3) {
                    _params.setInputMode(static_cast<InputSourceMode>(src));
                }
            }
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "bt_cmd") == 0) {
            const char* cmd = doc["cmd"] | "";
            if (strlen(cmd) > 0) {
                Serial1.println(cmd);
                Serial.printf("[WEB] Forwarded AVRC cmd to WROOM: %s\n", cmd);
            }
        }
        else if (strcmp(t, "bt_meta_test") == 0) {
            BluetoothMetadata meta;
            strncpy(meta.title, "Blinding Lights", sizeof(meta.title) - 1);
            strncpy(meta.artist, "The Weeknd", sizeof(meta.artist) - 1);
            strncpy(meta.album, "After Hours", sizeof(meta.album) - 1);
            meta.connected = true;
            meta.playing = true;
            _params.setBluetoothMetadata(meta);
            _ws.textAll(_buildBtMetaJson(meta));
        }
        else if (strcmp(t, "boot_defaults") == 0) {
            bool custom   = doc.containsKey("custom") ? doc["custom"].as<bool>() : false;
            bool eq       = doc.containsKey("eq") ? doc["eq"].as<bool>() : false;
            bool subsonic = doc.containsKey("subsonic") ? doc["subsonic"].as<bool>() : (doc.containsKey("sub") ? doc["sub"].as<bool>() : false);
            bool comp     = doc.containsKey("comp") ? doc["comp"].as<bool>() : false;
            bool lim      = doc.containsKey("lim") ? doc["lim"].as<bool>() : false;
            _params.setBootDefaults(custom, eq, comp, lim, subsonic);
            _params.save();
            _ws.textAll(_buildStateJson());
        }
        else if (strcmp(t, "module_order") == 0) {
            String orderStr = "";
            if (doc["order"].is<JsonArray>()) {
                JsonArray arr = doc["order"].as<JsonArray>();
                for (JsonVariant v : arr) {
                    const char* name = v.as<const char*>();
                    if (name && strlen(name) > 0) {
                        if (orderStr.length() > 0) orderStr += ",";
                        orderStr += name;
                    }
                }
            } else if (doc["order"].is<const char*>()) {
                orderStr = doc["order"].as<const char*>();
            }
            if (orderStr.length() > 0) {
                _params.setModuleOrder(orderStr.c_str());
                _params.save();
                _ws.textAll(_buildStateJson());
            }
        }
        else if (strcmp(t, "getState") == 0) {
            client->text(_buildStateJson());
        }
    }

    // -- EQ message handler -----------------------------------
    void _handleEQMsg(const JsonDocument& doc) {
        const int band = doc["band"] | -1;
        if (band < 0 || band >= kEQBands) return;

        EQBandParams p = _params.getEQBand(band);
        if (doc.containsKey("gain"))    p.gain    = doc["gain"].as<float>();
        if (doc.containsKey("q"))       p.q       = doc["q"].as<float>();
        if (doc.containsKey("freq"))    p.freq    = doc["freq"].as<float>();
        if (doc.containsKey("enabled")) p.enabled = doc["enabled"].as<bool>();
        if (doc.containsKey("type"))    p.type    = doc["type"].as<uint8_t>();
        if (doc.containsKey("order"))   p.order   = doc["order"].as<uint8_t>();

        p.gain = _clamp(p.gain, -18.0f,  18.0f);
        p.q    = _clamp(p.q,     0.05f,   10.0f);
        p.freq = _clamp(p.freq,  20.0f,  20000.0f);
        if (p.order < 1 || p.order > 4) p.order = 2;
        if (p.type == 5) {
            p.type  = EQ_FILTER_LOW_PASS;
            p.order = 4;
        } else if (p.type == 6) {
            p.type  = EQ_FILTER_HIGH_PASS;
            p.order = 4;
        } else if (p.type > 4) {
            p.type = 0;
        }

        _params.setEQBand(band, p);
    }

    // -- Speakers correction (Subsonic + Channel correction) message handler --
    void _handleSubsonicMsg(const JsonDocument& doc) {
        SubsonicParams p = _params.getSubsonic();
        if (doc.containsKey("freq"))    p.freq     = doc["freq"].as<float>();
        if (doc.containsKey("order"))   p.order    = doc["order"].as<uint8_t>();
        if (doc.containsKey("enabled")) p.enabled  = doc["enabled"].as<bool>();
        if (doc.containsKey("balance")) p.balance  = doc["balance"].as<float>();
        if (doc.containsKey("delayL"))  p.delayMsL = doc["delayL"].as<float>();
        if (doc.containsKey("delayR"))  p.delayMsR = doc["delayR"].as<float>();
        if (doc.containsKey("phaseL"))  p.phaseL   = doc["phaseL"].as<int16_t>();
        if (doc.containsKey("phaseR"))  p.phaseR   = doc["phaseR"].as<int16_t>();

        p.freq = _clamp(p.freq, 20.0f, 100.0f);
        if (p.order < 1 || p.order > 4) p.order = 2;
        p.balance  = _clamp(p.balance, -100.0f, 100.0f);
        p.delayMsL = _clamp(p.delayMsL, 0.0f, 20.0f);
        p.delayMsR = _clamp(p.delayMsR, 0.0f, 20.0f);
        if (p.phaseL < 0 || p.phaseL > 180) p.phaseL = 0;
        if (p.phaseR < 0 || p.phaseR > 180) p.phaseR = 0;

        _params.setSubsonic(p);
    }

    // -- Compander message handler ----------------------------
    void _handleCompanderMsg(const JsonDocument& doc) {
        CompanderParams p = _params.getCompander();
        if (doc.containsKey("threshold")) p.threshold  = doc["threshold"].as<float>();
        if (doc.containsKey("ratio"))     p.ratio      = doc["ratio"].as<float>();
        if (doc.containsKey("attackMs"))  p.attackMs   = doc["attackMs"].as<float>();
        if (doc.containsKey("releaseMs")) p.releaseMs  = doc["releaseMs"].as<float>();
        if (doc.containsKey("makeup"))    p.makeupGain = doc["makeup"].as<float>();
        if (doc.containsKey("enabled"))   p.enabled    = doc["enabled"].as<bool>();
        _params.setCompander(p);
    }

    // -- Limiter message handler ------------------------------
    void _handleLimiterMsg(const JsonDocument& doc) {
        LimiterParams p = _params.getLimiter();
        if (doc.containsKey("ceiling")) p.ceilingDb = doc["ceiling"].as<float>();
        if (doc.containsKey("knee"))    p.kneeDb    = doc["knee"].as<float>();
        if (doc.containsKey("enabled")) p.enabled   = doc["enabled"].as<bool>();
        _params.setLimiter(p);
    }

    // -- JSON builder -----------------------------------------
    String _buildStateJson() {
        DynamicJsonDocument doc(4096);
        doc["t"] = "state";

        // Input Source Module
        JsonObject inObj = doc.createNestedObject("input");
        inObj["source"] = static_cast<uint8_t>(_params.getInputMode());
        inObj["active"] = static_cast<uint8_t>(_params.getActiveSource());

        // EQ Module
        JsonObject eqObj = doc.createNestedObject("eq");
        eqObj["enabled"] = _params.getEQEnabled();
        eqObj["preamp"]  = roundf(_params.getPreamp() * 100.0f) / 100.0f;
        int count = _params.getEQBandCount();
        eqObj["count"]   = count;
        JsonArray eqBands = eqObj.createNestedArray("bands");
        for (int b = 0; b < count; ++b) {
            EQBandParams p = _params.getEQBand(b);
            JsonObject o   = eqBands.createNestedObject();
            o["band"]    = b;
            o["gain"]    = roundf(p.gain * 100.0f) / 100.0f;
            o["q"]       = roundf(p.q * 1000.0f) / 1000.0f;
            o["freq"]    = roundf(p.freq * 10.0f) / 10.0f;
            o["type"]    = p.type;
            o["order"]   = p.order;
            o["enabled"] = p.enabled;
        }

        // Speakers Correction Module
        auto sub = _params.getSubsonic();
        JsonObject subObj = doc.createNestedObject("subsonic");
        subObj["enabled"] = sub.enabled;
        subObj["freq"]    = roundf(sub.freq * 10.0f) / 10.0f;
        subObj["order"]   = sub.order;
        subObj["balance"] = roundf(sub.balance * 10.0f) / 10.0f;
        subObj["delayL"]  = roundf(sub.delayMsL * 10.0f) / 10.0f;
        subObj["delayR"]  = roundf(sub.delayMsR * 10.0f) / 10.0f;
        subObj["phaseL"]  = sub.phaseL;
        subObj["phaseR"]  = sub.phaseR;

        // Compander Module
        auto comp = _params.getCompander();
        JsonObject cObj = doc.createNestedObject("compander");
        cObj["enabled"]   = comp.enabled;
        cObj["threshold"] = roundf(comp.threshold * 10.0f) / 10.0f;
        cObj["ratio"]     = roundf(comp.ratio * 10.0f) / 10.0f;
        cObj["attackMs"]  = roundf(comp.attackMs * 10.0f) / 10.0f;
        cObj["releaseMs"] = roundf(comp.releaseMs * 10.0f) / 10.0f;
        cObj["makeup"]    = roundf(comp.makeupGain * 10.0f) / 10.0f;

        // Limiter Module
        auto lim = _params.getLimiter();
        JsonObject lObj = doc.createNestedObject("limiter");
        lObj["enabled"] = lim.enabled;
        lObj["ceiling"] = roundf(lim.ceilingDb * 10.0f) / 10.0f;
        lObj["knee"]    = roundf(lim.kneeDb * 10.0f) / 10.0f;

        // Boot Defaults Configuration
        bool bCustom, bEQ, bComp, bLim, bSub;
        _params.getBootDefaults(bCustom, bEQ, bComp, bLim, bSub);
        JsonObject bootObj = doc.createNestedObject("boot_defaults");
        bootObj["custom"]   = bCustom;
        bootObj["eq"]       = bEQ;
        bootObj["subsonic"] = bSub;
        bootObj["sub"]      = bSub;
        bootObj["comp"]     = bComp;
        bootObj["lim"]      = bLim;

        // Module Execution Order
        String orderStr = _params.getModuleOrder();
        JsonArray orderArr = doc.createNestedArray("module_order");
        int startIdx = 0;
        while (startIdx < orderStr.length()) {
            int comma = orderStr.indexOf(',', startIdx);
            if (comma < 0) comma = orderStr.length();
            String item = orderStr.substring(startIdx, comma);
            item.trim();
            if (item.length() > 0) orderArr.add(item);
            startIdx = comma + 1;
        }

        // Bluetooth Metadata Module
        auto bt = _params.getBluetoothMetadata();
        JsonObject btObj = doc.createNestedObject("bt");
        btObj["title"]     = bt.title;
        btObj["artist"]    = bt.artist;
        btObj["album"]     = bt.album;
        btObj["connected"] = bt.connected;
        btObj["playing"]   = bt.playing;
        btObj["duration"]  = bt.duration;
        btObj["pos"]       = bt.pos;

        // Wi-Fi Stream Metadata Module (LMS / Roon)
        auto wf = _params.getWiFiStreamMetadata();
        JsonObject wfObj = doc.createNestedObject("wifi_stream");
        wfObj["server"]    = wf.serverName;
        wfObj["server_ip"] = wf.serverIp;
        wfObj["title"]     = wf.trackTitle;
        wfObj["artist"]    = wf.trackArtist;
        wfObj["album"]     = wf.trackAlbum;
        wfObj["format"]    = wf.formatInfo;
        wfObj["connected"] = wf.connected;
        wfObj["streaming"] = wf.streaming;
        wfObj["paused"]    = wf.paused;
        wfObj["rate"]      = wf.sampleRate;
        wfObj["bits"]      = wf.bitDepth;

        // System Telemetry (CPU 0/1, RAM, Flash)
        auto s = SystemMonitor::getStats();
        JsonObject sysObj = doc.createNestedObject("sys");
        sysObj["cpu0"]    = roundf(s.cpu0 * 10.0f) / 10.0f;
        sysObj["cpu1"]    = roundf(s.cpu1 * 10.0f) / 10.0f;
        sysObj["heap_u"]  = s.heapUsed;
        sysObj["heap_t"]  = s.heapTotal;
        sysObj["psram_u"] = s.psramUsed;
        sysObj["psram_t"] = s.psramTotal;
        sysObj["flash_u"] = s.flashUsed;
        sysObj["flash_t"] = s.flashTotal;

        String out;
        serializeJson(doc, out);
        return out;
    }

    String _buildSysJson() {
        auto s = SystemMonitor::getStats();
        StaticJsonDocument<256> doc;
        doc["t"]       = "sys";
        doc["cpu0"]    = roundf(s.cpu0 * 10.0f) / 10.0f;
        doc["cpu1"]    = roundf(s.cpu1 * 10.0f) / 10.0f;
        doc["heap_u"]  = s.heapUsed;
        doc["heap_t"]  = s.heapTotal;
        doc["psram_u"] = s.psramUsed;
        doc["psram_t"] = s.psramTotal;
        doc["flash_u"] = s.flashUsed;
        doc["flash_t"] = s.flashTotal;

        String out;
        serializeJson(doc, out);
        return out;
    }

    String _buildBtMetaJson(const BluetoothMetadata& bt) {
        StaticJsonDocument<384> doc;
        doc["t"]         = "bt_meta";
        doc["title"]     = bt.title;
        doc["artist"]    = bt.artist;
        doc["album"]     = bt.album;
        doc["connected"] = bt.connected;
        doc["playing"]   = bt.playing;
        doc["duration"]  = bt.duration;
        doc["pos"]       = bt.pos;
        String out;
        serializeJson(doc, out);
        return out;
    }

    String _buildWiFiMetaJson(const WiFiStreamMetadata& wf) {
        StaticJsonDocument<384> doc;
        doc["t"]         = "wifi_meta";
        doc["server"]    = wf.serverName;
        doc["server_ip"] = wf.serverIp;
        doc["title"]     = wf.trackTitle;
        doc["artist"]    = wf.trackArtist;
        doc["album"]     = wf.trackAlbum;
        doc["format"]    = wf.formatInfo;
        doc["connected"] = wf.connected;
        doc["streaming"] = wf.streaming;
        doc["paused"]    = wf.paused;
        doc["rate"]      = wf.sampleRate;
        doc["bits"]      = wf.bitDepth;
        String out;
        serializeJson(doc, out);
        return out;
    }

    void _processUartLine(const String& line) {
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, line) == DeserializationError::Ok) {
            const char* t = doc["t"];
            if (t && strcmp(t, "bt_meta") == 0) {
                BluetoothMetadata meta;
                const char* title  = doc["title"]  | "No Track Playing";
                const char* artist = doc["artist"] | "Unknown Artist";
                const char* album  = doc["album"]  | "Unknown Album";
                strncpy(meta.title, title, sizeof(meta.title) - 1);
                meta.title[sizeof(meta.title) - 1] = '\0';
                strncpy(meta.artist, artist, sizeof(meta.artist) - 1);
                meta.artist[sizeof(meta.artist) - 1] = '\0';
                strncpy(meta.album, album, sizeof(meta.album) - 1);
                meta.album[sizeof(meta.album) - 1] = '\0';
                meta.connected = doc["connected"] | false;
                meta.playing   = doc["playing"]   | false;
                meta.duration  = doc["duration"]  | 0;
                meta.pos       = doc["pos"]       | 0;
                _params.setBluetoothMetadata(meta);

                // Broadcast directly to connected WebSockets
                _ws.textAll(line);
                Serial.printf("[WEB] BT Track: %s - %s (%s)\n", meta.title, meta.artist, meta.album);
            }
        }
    }

    static float _clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};
