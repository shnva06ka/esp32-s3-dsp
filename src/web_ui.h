#pragma once

#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <cstring>
#include "eq_html.h"
#include "console_html.h"
#include "wireless_console.h"
#include "ota_manager.h"
#include "param_store.h"
#include "sys_monitor.h"
#include "dsp_module_id.h"

#if __has_include("wifi_credentials.h")
#  include "wifi_credentials.h"
#endif

#ifndef WIFI_SSID
#  define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#  define WIFI_PASS ""
#endif

class WebUI {
public:
    static WebUI*& getInstance() {
        static WebUI* instance = nullptr;
        return instance;
    }

    explicit WebUI(ParamStore& params)
        : _params(params),
          _server(80),
          _ws("/ws")
    {}

    void begin() {
        getInstance() = this;

        _startWiFi();

        Console.begin();
        Console.setBroadcastFunc([](const char* msg) {
            WebUI* inst = getInstance();
            if (inst && inst->_ws.count() > 0) {
                StaticJsonDocument<512> doc;
                doc["t"] = "log";
                doc["msg"] = msg;
                String out;
                serializeJson(doc, out);
                inst->_ws.textAll(out);
            }
        });

        _setupRoutes();
        _server.addHandler(&_ws);
        _server.begin();
        Serial.println("[WEB] HTTP & WebSocket server started");

        OTAManager::beginArduinoOTA();
    }

    void loop() {
        Console.loop();
        OTAManager::loopArduinoOTA();
        _ws.cleanupClients();

        InputSourceMode mode;
        ActiveInputSource active;
        if (_params.getInputStateIfChanged(mode, active, _lastInputVersion)) {
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }

        if (_stateBroadcastPending.exchange(false, std::memory_order_acq_rel)) {
            if (_ws.count() > 0) {
                _ws.textAll(_buildStateJson());
            }
        }

        WiFiStreamMetadata wifiMeta;
        if (_params.getWiFiStreamMetadataIfChanged(wifiMeta, _lastWifiMetaVersion)) {
            if (_ws.count() > 0) {
                _ws.textAll(_buildWiFiMetaJson(wifiMeta));
            }
        }

        static uint32_t lastPsCheck = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastPsCheck >= 5000) {
            lastPsCheck = nowMs;
            if (WiFi.status() == WL_CONNECTED) {
                esp_wifi_set_ps(WIFI_PS_NONE);
            }
        }

        unsigned long now = millis();
        if (now - _lastIpPrint >= 30000) {
            _lastIpPrint = now;
            IPAddress ip = (WiFi.getMode() & WIFI_MODE_STA && WiFi.status() == WL_CONNECTED)
                             ? WiFi.localIP()
                             : WiFi.softAPIP();
            Serial.printf("[NET] Web UI accessible at: http://%u.%u.%u.%u (or http://dsp.local)\n", ip[0], ip[1], ip[2], ip[3]);
        }

        if (_params.isDirty()) {
            if (now - _params.getLastDirtyTime() >= 1000) {
                _params.save();
            }
        }

        if (now - _lastSysBroadcast >= 1000) {
            _lastSysBroadcast = now;
            if (_ws.count() > 0) {
                _ws.textAll(_buildSysJson());
            }
        }

        if (_wifiReconnectRequested && now >= _wifiReconnectTimer) {
            _performWiFiReconnect();
        }
    }

private:
    ParamStore&       _params;
    AsyncWebServer    _server;
    AsyncWebSocket    _ws;
    unsigned long     _lastIpPrint{0};
    unsigned long     _lastSysBroadcast{0};
    uint32_t          _lastInputVersion{1};
    uint32_t          _lastWifiMetaVersion{1};
    std::atomic<bool> _stateBroadcastPending{false};
    bool              _wifiReconnectRequested{false};
    String            _newSSID{""};
    String            _newPass{""};
    unsigned long     _wifiReconnectTimer{0};

    void _startWiFi() {
        Preferences prefs;
        String savedSSID = "";
        String savedPass = "";
        if (prefs.begin("wifi_cfg", true)) {
            savedSSID = prefs.getString("ssid", "");
            savedPass = prefs.getString("pass", "");
            prefs.end();
        }

        String targetSSID = savedSSID.length() > 0 ? savedSSID : String(WIFI_SSID);
        String targetPass = savedSSID.length() > 0 ? savedPass : String(WIFI_PASS);

        bool hasCredentials = (targetSSID.length() > 0 && targetSSID != "YourSSID");

        if (hasCredentials) {
            WiFi.mode(WIFI_STA);
            WiFi.setAutoReconnect(true);
            WiFi.setSleep(false);
            esp_wifi_set_ps(WIFI_PS_NONE);
            WiFi.setTxPower(WIFI_POWER_19_5dBm);
            WiFi.begin(targetSSID.c_str(), targetPass.c_str());
            Serial.printf("[WEB] Connecting to Wi-Fi SSID '%s'", targetSSID.c_str());
            for (uint8_t i = 0; i < 40 && WiFi.status() != WL_CONNECTED; ++i) {
                delay(500);
                Serial.print('.');
            }
            if (WiFi.status() == WL_CONNECTED) {
                WiFi.setSleep(false);
                esp_wifi_set_ps(WIFI_PS_NONE);
                Serial.printf("\n[WEB] Wi-Fi connected! Station IP: %s (Power Save DISABLED)\n", WiFi.localIP().toString().c_str());
                if (MDNS.begin("dsp")) {
                    MDNS.addService("http", "tcp", 80);
                    Serial.println("[NET] mDNS responder active: http://dsp.local");
                }
                return;
            }
            Serial.println("\n[WEB] Wi-Fi connection timed out");
        }

        Serial.println("[WEB] Starting fallback AP 'ESP32-S3-DSP'...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32-S3-DSP");
        Serial.printf("[WEB] Fallback AP active! Network SSID: 'ESP32-S3-DSP' | AP IP: %s\n",
                      WiFi.softAPIP().toString().c_str());
        if (MDNS.begin("dsp")) {
            MDNS.addService("http", "tcp", 80);
        }
        Serial.printf("[NET] Web UI accessible at: http://%s (or http://dsp.local)\n", WiFi.softAPIP().toString().c_str());
    }

    void _performWiFiReconnect() {
        _wifiReconnectRequested = false;
        Serial.printf("\n[WEB] Attempting Wi-Fi connection to SSID '%s'...\n", _newSSID.c_str());

        WiFi.mode(WIFI_AP_STA);
        WiFi.setSleep(false);
        esp_wifi_set_ps(WIFI_PS_NONE);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.begin(_newSSID.c_str(), _newPass.c_str());

        for (uint8_t i = 0; i < 25 && WiFi.status() != WL_CONNECTED; ++i) {
            delay(400);
            Serial.print('.');
        }

        if (WiFi.status() == WL_CONNECTED) {
            WiFi.setSleep(false);
            esp_wifi_set_ps(WIFI_PS_NONE);
            Serial.printf("\n[WEB] Connected to '%s'! Station IP: %s (Power Save DISABLED)\n", _newSSID.c_str(), WiFi.localIP().toString().c_str());

            Preferences prefs;
            if (prefs.begin("wifi_cfg", false)) {
                prefs.putString("ssid", _newSSID);
                prefs.putString("pass", _newPass);
                prefs.end();
            }

            MDNS.end();
            if (MDNS.begin("dsp")) {
                MDNS.addService("http", "tcp", 80);
                Serial.println("[NET] mDNS active: http://dsp.local");
            }

            StaticJsonDocument<256> doc;
            doc["t"] = "wifi_status";
            doc["status"] = "connected";
            doc["ssid"] = _newSSID;
            doc["ip"] = WiFi.localIP().toString();
            doc["msg"] = "Connected successfully to " + _newSSID;
            String out;
            serializeJson(doc, out);
            _ws.textAll(out);
        } else {
            Serial.println("\n[WEB] Wi-Fi connection failed! Remaining in current mode.");
            StaticJsonDocument<256> doc;
            doc["t"] = "wifi_status";
            doc["status"] = "failed";
            doc["msg"] = "Could not connect to '" + _newSSID + "'. Please check password and try again.";
            String out;
            serializeJson(doc, out);
            _ws.textAll(out);
        }
    }

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

        _server.on("/api/wifi", HTTP_POST, [this](AsyncWebServerRequest* req) {
            String ssid = req->hasParam("ssid", true) ? req->getParam("ssid", true)->value() : "";
            String pass = req->hasParam("pass", true) ? req->getParam("pass", true)->value() : "";
            if (ssid.length() > 0) {
                _newSSID = ssid;
                _newPass = pass;
                _wifiReconnectRequested = true;
                _wifiReconnectTimer = millis() + 300;
                req->send(200, "application/json", "{\"status\":\"connecting\",\"msg\":\"Connecting to network...\"}");
            } else {
                req->send(400, "application/json", "{\"error\":\"Missing SSID\"}");
            }
        });

        _server.on("/api/wifi", HTTP_GET, [](AsyncWebServerRequest* req) {
            StaticJsonDocument<256> doc;
            doc["connected"] = (WiFi.status() == WL_CONNECTED);
            doc["ssid"] = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : (WiFi.getMode() == WIFI_AP ? "ESP32-S3-DSP" : "");
            doc["ip"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
            doc["mode"] = (WiFi.status() == WL_CONNECTED) ? "sta" : (WiFi.getMode() == WIFI_AP ? "ap" : "other");
            doc["rssi"] = WiFi.RSSI();
            String out;
            serializeJson(doc, out);
            req->send(200, "application/json", out);
        });

        OTAManager::setupWebRoutes(_server);

        _server.on("/console", HTTP_GET, [](AsyncWebServerRequest* req) {
            req->send(200, "text/html", s_consoleHtml);
        });

        _server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* req) {
            req->send(200, "text/plain", Console.getRecentLogs());
        });

        _server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* req) {
            req->send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Rebooting ESP32-S3...\"}");
            delay(250);
            ESP.restart();
        });

        _server.onNotFound([](AsyncWebServerRequest* req) {
            req->send(404, "text/plain", "Not found");
        });
    }

    void _onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT:
                Serial.printf("[WS] Client #%u connected from %s\n",
                              client->id(), client->remoteIP().toString().c_str());
                client->setCloseClientOnQueueFull(false);
                client->text(_buildStateJson());
                client->text(_buildSysJson());
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

    void _handleWsMessage(AsyncWebSocketClient* client, const char* json, size_t len) {
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, json, len) != DeserializationError::Ok) return;

        const char* t = doc["t"] | "";

        if (strcmp(t, "eq") == 0 || strcmp(t, "eq_band") == 0) {
            _handleEQMsg(doc);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "eq_toggle") == 0 || strcmp(t, "eq_module") == 0) {
            if (doc.containsKey("enabled")) {
                _params.setEQEnabled(doc["enabled"].as<bool>());
            }
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "eq_preamp") == 0) {
            if (doc.containsKey("preamp")) {
                _params.setPreamp(doc["preamp"].as<float>());
            }
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "eq_preset") == 0) {
            const char* preset = doc["preset"] | doc["name"] | "flat";
            _params.applyPreset(preset);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "add_band") == 0) {
            float f = doc["freq"] | 1000.0f;
            float g = doc["gain"] | 0.0f;
            float q = doc["q"]    | 0.707f;
            uint8_t type = doc["type"] | 0;
            uint8_t order = doc["order"] | 2;
            _params.addEQBand(f, g, q, type, order);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "remove_band") == 0) {
            int b = doc["band"] | -1;
            _params.removeEQBand(b);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "subsonic") == 0) {
            _handleSubsonicMsg(doc);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "compander") == 0) {
            _handleCompanderMsg(doc);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "limiter") == 0) {
            _handleLimiterMsg(doc);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "input_source") == 0 || strcmp(t, "input") == 0) {
            if (doc.containsKey("source")) {
                uint8_t src = doc["source"].as<uint8_t>();
                if (src == 1 || src == 2 || src == 3) {
                    _params.setInputMode(static_cast<InputSourceMode>(src));
                }
            }
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "wifi_save") == 0 || strcmp(t, "wifi_config") == 0) {
            const char* ssid = doc["ssid"] | "";
            const char* pass = doc["pass"] | "";
            if (strlen(ssid) > 0) {
                _newSSID = ssid;
                _newPass = pass;
                _wifiReconnectRequested = true;
                _wifiReconnectTimer = millis() + 300;
                client->text("{\"t\":\"wifi_status\",\"status\":\"connecting\",\"msg\":\"Initiating Wi-Fi connection...\"}");
            }
        }
        else if (strcmp(t, "boot_defaults") == 0) {
            bool custom   = doc.containsKey("custom") ? doc["custom"].as<bool>() : false;
            bool eq       = doc.containsKey("eq") ? doc["eq"].as<bool>() : false;
            bool subsonic = doc.containsKey("subsonic") ? doc["subsonic"].as<bool>() : (doc.containsKey("sub") ? doc["sub"].as<bool>() : false);
            bool comp     = doc.containsKey("comp") ? doc["comp"].as<bool>() : false;
            bool lim      = doc.containsKey("lim") ? doc["lim"].as<bool>() : false;
            _params.setBootDefaults(custom, eq, comp, lim, subsonic);
            _stateBroadcastPending.store(true, std::memory_order_relaxed);
        }
        else if (strcmp(t, "module_order") == 0) {
            String orderStr = "";
            if (doc["order"].is<JsonArray>()) {
                JsonArray arr = doc["order"].as<JsonArray>();
                for (JsonVariant v : arr) {
                    const char* name = v.as<const char*>();
                    DspModuleId module = parseDspModuleId(name);
                    if (module != DspModuleId::Unknown) {
                        if (orderStr.length() > 0) orderStr += ",";
                        orderStr += toDspModuleToken(module);
                    }
                }
            } else if (doc["order"].is<const char*>()) {
                const char* rawOrder = doc["order"].as<const char*>();
                char tmp[64];
                strncpy(tmp, rawOrder ? rawOrder : "", sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = '\0';
                char* token = strtok(tmp, ",");
                while (token) {
                    while (*token == ' ') token++;
                    DspModuleId module = parseDspModuleId(token);
                    if (module != DspModuleId::Unknown) {
                        if (orderStr.length() > 0) orderStr += ",";
                        orderStr += toDspModuleToken(module);
                    }
                    token = strtok(nullptr, ",");
                }
            }
            if (orderStr.length() > 0) {
                _params.setModuleOrder(orderStr.c_str());
                _stateBroadcastPending.store(true, std::memory_order_relaxed);
            }
        }
        else if (strcmp(t, "getState") == 0) {
            client->text(_buildStateJson());
        }
    }

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

    void _handleLimiterMsg(const JsonDocument& doc) {
        LimiterParams p = _params.getLimiter();
        if (doc.containsKey("ceiling")) p.ceilingDb = doc["ceiling"].as<float>();
        if (doc.containsKey("knee"))    p.kneeDb    = doc["knee"].as<float>();
        if (doc.containsKey("enabled")) p.enabled   = doc["enabled"].as<bool>();
        _params.setLimiter(p);
    }

    String _buildStateJson() {
        StaticJsonDocument<4096> doc;
        doc["t"] = "state";

        JsonObject inObj = doc.createNestedObject("input");
        inObj["source"] = static_cast<uint8_t>(_params.getInputMode());
        inObj["active"] = static_cast<uint8_t>(_params.getActiveSource());

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

        auto comp = _params.getCompander();
        JsonObject cObj = doc.createNestedObject("compander");
        cObj["enabled"]   = comp.enabled;
        cObj["threshold"] = roundf(comp.threshold * 10.0f) / 10.0f;
        cObj["ratio"]     = roundf(comp.ratio * 10.0f) / 10.0f;
        cObj["attackMs"]  = roundf(comp.attackMs * 10.0f) / 10.0f;
        cObj["releaseMs"] = roundf(comp.releaseMs * 10.0f) / 10.0f;
        cObj["makeup"]    = roundf(comp.makeupGain * 10.0f) / 10.0f;

        auto lim = _params.getLimiter();
        JsonObject lObj = doc.createNestedObject("limiter");
        lObj["enabled"] = lim.enabled;
        lObj["ceiling"] = roundf(lim.ceilingDb * 10.0f) / 10.0f;
        lObj["knee"]    = roundf(lim.kneeDb * 10.0f) / 10.0f;

        bool bCustom, bEQ, bComp, bLim, bSub;
        _params.getBootDefaults(bCustom, bEQ, bComp, bLim, bSub);
        JsonObject bootObj = doc.createNestedObject("boot_defaults");
        bootObj["custom"]   = bCustom;
        bootObj["eq"]       = bEQ;
        bootObj["subsonic"] = bSub;
        bootObj["sub"]      = bSub;
        bootObj["comp"]     = bComp;
        bootObj["lim"]      = bLim;

        String orderStr = _params.getModuleOrder();
        JsonArray orderArr = doc.createNestedArray("module_order");
        int startIdx = 0;
        while (startIdx < orderStr.length()) {
            int comma = orderStr.indexOf(',', startIdx);
            if (comma < 0) comma = orderStr.length();
            String item = orderStr.substring(startIdx, comma);
            item.trim();
            if (item.length() > 0) {
                DspModuleId module = parseDspModuleId(item.c_str());
                if (module != DspModuleId::Unknown) {
                    orderArr.add(toDspModuleToken(module));
                }
            }
            startIdx = comma + 1;
        }

        auto wf = _params.getWiFiStreamMetadata();
        JsonObject wfObj = doc.createNestedObject("wifi_stream");
        wfObj["server"]    = wf.serverName;
        wfObj["server_ip"] = wf.serverIp;
        wfObj["format"]    = wf.formatInfo;
        wfObj["connected"] = wf.connected;
        wfObj["streaming"] = wf.streaming;
        wfObj["paused"]    = wf.paused;
        wfObj["rate"]      = wf.sampleRate;
        wfObj["bits"]      = wf.bitDepth;

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
        sysObj["idle0"]    = s.idleRate0;
        sysObj["idle1"]    = s.idleRate1;
        sysObj["idle_base0"] = s.idleBaseline0;
        sysObj["idle_base1"] = s.idleBaseline1;

        String out;
        serializeJson(doc, out);
        return out;
    }

    String _buildSysJson() {
        auto s = SystemMonitor::getStats();
        StaticJsonDocument<384> doc;
        doc["t"]         = "sys";
        doc["cpu0"]      = roundf(s.cpu0 * 10.0f) / 10.0f;
        doc["cpu1"]      = roundf(s.cpu1 * 10.0f) / 10.0f;
        doc["heap_u"]    = s.heapUsed;
        doc["heap_t"]    = s.heapTotal;
        doc["psram_u"]   = s.psramUsed;
        doc["psram_t"]   = s.psramTotal;
        doc["flash_u"]   = s.flashUsed;
        doc["flash_t"]   = s.flashTotal;
        doc["idle0"]     = s.idleRate0;
        doc["idle1"]     = s.idleRate1;
        doc["idle_base0"] = s.idleBaseline0;
        doc["idle_base1"] = s.idleBaseline1;
        doc["wifi_con"]  = (WiFi.status() == WL_CONNECTED);
        doc["wifi_ssid"] = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : (WiFi.getMode() == WIFI_AP ? "ESP32-S3-DSP (AP)" : "Disconnected");
        doc["wifi_ip"]   = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();

        String out;
        serializeJson(doc, out);
        return out;
    }

    String _buildWiFiMetaJson(const WiFiStreamMetadata& wf) {
        StaticJsonDocument<256> doc;
        doc["t"]         = "wifi_meta";
        doc["server"]    = wf.serverName;
        doc["server_ip"] = wf.serverIp;
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

    static float _clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
};
