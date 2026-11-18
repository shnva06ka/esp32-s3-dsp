#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <WiFi.h>
#include <cstring>
#include <esp_freertos_hooks.h>
#include <esp_netif.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "lms_client.h"
#include "param_store.h"
#include "wireless_console.h"

// === src/ap_fallback_policy.h ===


esp_netif_t* get_esp_interface_netif(esp_interface_t interface);

inline const char* const* apCaptivePortalProbePaths(size_t& outCount) {
    static const char* const kProbePaths[] = {
        "/generate_204",
        "/gen_204",
        "/hotspot-detect.html",
        "/library/test/success.html",
        "/connecttest.txt",
        "/ncsi.txt",
        "/redirect",
        "/canonical.html",
        "/success.txt",
    };
    outCount = sizeof(kProbePaths) / sizeof(kProbePaths[0]);
    return kProbePaths;
}

inline bool apIsCaptivePortalProbePath(const char* path) {
    if (!path || !path[0]) return false;

    size_t count = 0;
    const char* const* probes = apCaptivePortalProbePaths(count);
    for (size_t i = 0; i < count; ++i) {
        const char* probe = probes[i];
        size_t probeLen = strlen(probe);
        if (strcmp(path, probe) == 0) {
            return true;
        }
        // Probe URL with query string
        if (strncmp(path, probe, probeLen) == 0 && path[probeLen] == '?') {
            return true;
        }
    }
    return false;
}

inline const char* apCaptivePortalRedirectUrl() {
    return "http://192.168.4.1/";
}

inline bool apIsLocalDspHostname(const char* host) {
    if (!host || !host[0]) {
        return false;
    }

    size_t len = strlen(host);
    if (len > 0 && host[len - 1] == '.') {
        --len;
    }
    if (len == 0 || len > 32) {
        return false;
    }

    char lower[33];
    for (size_t i = 0; i < len; ++i) {
        unsigned char c = static_cast<unsigned char>(host[i]);
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<unsigned char>(c - 'A' + 'a');
        }
        lower[i] = static_cast<char>(c);
    }
    lower[len] = '\0';

    static const char* const kLocalHosts[] = {
        "dsp",
        "dsp.local",
        "esp32-s3-dsp",
        "esp32-s3-dsp.local",
    };
    for (const char* candidate : kLocalHosts) {
        if (strcmp(lower, candidate) == 0) {
            return true;
        }
    }
    return false;
}

inline size_t apBuildCaptiveDnsResponse(const uint8_t* req, size_t reqLen,
                                        const IPAddress& apIp,
                                        uint8_t* outBuf, size_t outCap) {
    static constexpr size_t kDnsHeaderLen = 12;
    static constexpr size_t kDnsAnswerALen = 16;

    if (!req || !outBuf || reqLen < kDnsHeaderLen || outCap < kDnsHeaderLen) {
        return 0;
    }

    // Standard queries only
    if ((req[2] & 0x80U) != 0 || (req[2] & 0x78U) != 0) {
        return 0;
    }
    const uint16_t qdCount = (static_cast<uint16_t>(req[4]) << 8) | req[5];
    if (qdCount < 1U) {
        return 0;
    }

    // QName bounds check
    char qname[64];
    size_t qnameLen = 0;
    bool qnameOverflow = false;
    size_t pos = kDnsHeaderLen;
    bool qnameComplete = false;
    while (pos < reqLen) {
        const uint8_t labelLen = req[pos];
        if (labelLen == 0) {
            ++pos;
            qnameComplete = true;
            break;
        }
        if ((labelLen & 0xC0U) == 0xC0U) {
            if (pos + 2U > reqLen) {
                return 0;
            }
            qnameOverflow = true;
            pos += 2U;
            qnameComplete = true;
            break;
        }
        if (labelLen > 63U) {
            return 0;
        }
        if (pos + 1U + labelLen >= reqLen) {
            return 0;
        }
        if (!qnameOverflow) {
            const size_t needDot = (qnameLen > 0U) ? 1U : 0U;
            if (qnameLen + needDot + labelLen >= sizeof(qname) ||
                memchr(req + pos + 1U, '\0', labelLen) != nullptr) {
                qnameOverflow = true;
            } else {
                if (needDot) {
                    qname[qnameLen++] = '.';
                }
                memcpy(qname + qnameLen, req + pos + 1U, labelLen);
                qnameLen += labelLen;
            }
        }
        pos += 1U + labelLen;
    }
    if (!qnameComplete || (pos + 4U) > reqLen) {
        return 0;
    }

    qname[qnameLen] = '\0';
    const bool isLocalHost = !qnameOverflow && apIsLocalDspHostname(qname);

    const uint16_t qtype = (static_cast<uint16_t>(req[pos]) << 8) | req[pos + 1];
    const uint16_t qclass = (static_cast<uint16_t>(req[pos + 2]) << 8) | req[pos + 3];
    const size_t qEnd = pos + 4U;

    // Local DSP A records, NXDOMAIN for external
    const bool wantAnswerA = isLocalHost && (qtype == 1U) && (qclass == 1U || qclass == 255U);
    const size_t respLen = qEnd + (wantAnswerA ? kDnsAnswerALen : 0U);
    if (respLen > outCap) {
        return 0;
    }

    if (outBuf != req) {
        memcpy(outBuf, req, qEnd);
    }

    outBuf[2] = static_cast<uint8_t>(0x84U | (req[2] & 0x01U)); // QR=1, AA=1, preserve RD
    outBuf[3] = isLocalHost ? 0x00U : 0x03U;                     // 0=NoError, 3=NXDOMAIN
    outBuf[4] = 0x00U;
    outBuf[5] = 0x01U;                                           // QDCount=1
    outBuf[6] = 0x00U;
    outBuf[7] = wantAnswerA ? 0x01U : 0x00U;                     // ANCount
    outBuf[8] = 0x00U;
    outBuf[9] = 0x00U;                                           // NSCount=0
    outBuf[10] = 0x00U;
    outBuf[11] = 0x00U;                                          // ARCount=0 (strip EDNS0 OPT)

    if (wantAnswerA) {
        uint8_t* ans = outBuf + qEnd;
        ans[0] = 0xC0U; ans[1] = 0x0CU;  // Name pointer to offset 12
        ans[2] = 0x00U; ans[3] = 0x01U;  // Type A
        ans[4] = 0x00U; ans[5] = 0x01U;  // Class IN
        ans[6] = 0x00U; ans[7] = 0x00U; ans[8] = 0x00U; ans[9] = 0x3CU; // TTL 60s
        ans[10] = 0x00U; ans[11] = 0x04U; // RDLen 4
        ans[12] = apIp[0];
        ans[13] = apIp[1];
        ans[14] = apIp[2];
        ans[15] = apIp[3];
    }

    return respLen;
}

inline esp_err_t apEnsureSoftApDhcpRunning() {
    esp_netif_t* apNetif = get_esp_interface_netif(ESP_IF_WIFI_AP);
    if (!apNetif) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_netif_dhcp_status_t dhcpStatus = ESP_NETIF_DHCP_INIT;
    esp_err_t err = esp_netif_dhcps_get_status(apNetif, &dhcpStatus);
    if (err != ESP_OK) {
        return err;
    }
    if (dhcpStatus != ESP_NETIF_DHCP_STARTED) {
        err = esp_netif_dhcps_start(apNetif);
    }
    return err;
}

inline bool apShouldSendWsTelemetry(bool clientConnected, bool canSend, size_t queueLen, size_t maxNonCriticalQueue = 8) {
    return clientConnected && canSend && (queueLen < maxNonCriticalQueue);
}

inline bool apIsStaAssociated() {
    wifi_ap_record_t ap_info{};
    return esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
}

inline void apEnterPureSoftApMode(const char* apSsid) {
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);

    // Clear STA credentials
    if (WiFi.getMode() & WIFI_MODE_STA) {
        esp_wifi_disconnect();
        wifi_config_t emptySta{};
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        esp_wifi_set_config(WIFI_IF_STA, &emptySta);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_config(WIFI_IF_STA, &emptySta);
    }

    WiFi.mode(WIFI_AP);
    delay(120); // Let arduino_events drain any already-queued STA_DISCONNECTED

    // Keep pure AP mode
    WiFi.setAutoReconnect(false);
    if (WiFi.getMode() != WIFI_AP) {
        if (WiFi.getMode() & WIFI_MODE_STA) {
            esp_wifi_disconnect();
        }
        WiFi.mode(WIFI_AP);
    }

    // SoftAP & DHCPS setup
    WiFi.softAP(apSsid, nullptr, 1, 0, 4, false);
    apEnsureSoftApDhcpRunning();
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_inactive_time(WIFI_IF_AP, 60);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

// === src/sys_monitor.h ===


class SystemMonitor {
private:
    struct State {
        portMUX_TYPE      mux = portMUX_INITIALIZER_UNLOCKED;
        volatile uint32_t idleCount0{0};
        volatile uint32_t idleCount1{0};
        uint32_t          maxIdleRate0{0};
        uint32_t          maxIdleRate1{0};
        uint32_t          idleRate0{0};
        uint32_t          idleRate1{0};
        unsigned long     lastSampleTime{0};
        float             cpu0Usage{0.0f};
        float             cpu1Usage{0.0f};
        uint32_t          heapTotal{0};
        uint32_t          psramTotal{0};
        uint32_t          flashUsed{0};
        uint32_t          flashTotal{0};
    };

    static State& _s() {
        static State s;
        return s;
    }

    static bool _idleHook0() {
        _s().idleCount0++;
        return false;
    }

    static bool _idleHook1() {
        _s().idleCount1++;
        return false;
    }

    static float _clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

public:
    struct Stats {
        float    cpu0;
        float    cpu1;
        float    tempC;
        uint32_t heapUsed;
        uint32_t heapTotal;
        uint32_t psramUsed;
        uint32_t psramTotal;
        uint32_t flashUsed;
        uint32_t flashTotal;
        uint32_t idleRate0;
        uint32_t idleRate1;
        uint32_t idleBaseline0;
        uint32_t idleBaseline1;
    };

    static void begin() {
        esp_register_freertos_idle_hook_for_cpu(_idleHook0, 0);
        esp_register_freertos_idle_hook_for_cpu(_idleHook1, 1);
        auto& st = _s();
        st.lastSampleTime = millis();
        st.heapTotal      = ESP.getHeapSize();
        st.psramTotal     = psramFound() ? ESP.getPsramSize() : 0;
        st.flashUsed      = ESP.getSketchSize();
        st.flashTotal     = ESP.getFlashChipSize();
    }

    static Stats getStats() {
        update();
        auto& st = _s();
        Stats s;
        portENTER_CRITICAL(&st.mux);
        s.cpu0          = st.cpu0Usage;
        s.cpu1          = st.cpu1Usage;
        s.heapTotal     = st.heapTotal;
        s.psramTotal    = st.psramTotal;
        s.flashUsed     = st.flashUsed;
        s.flashTotal    = st.flashTotal;
        s.idleRate0     = st.idleRate0;
        s.idleRate1     = st.idleRate1;
        s.idleBaseline0 = st.maxIdleRate0;
        s.idleBaseline1 = st.maxIdleRate1;
        portEXIT_CRITICAL(&st.mux);
        s.tempC      = temperatureRead();
        s.heapUsed   = s.heapTotal - ESP.getFreeHeap();
        s.psramUsed  = (s.psramTotal > 0) ? (s.psramTotal - ESP.getFreePsram()) : 0;
        return s;
    }

    static void update() {
        auto& st = _s();
        portENTER_CRITICAL(&st.mux);
        unsigned long now = millis();
        unsigned long elapsed = now - st.lastSampleTime;
        if (elapsed < 500) {
            portEXIT_CRITICAL(&st.mux);
            return;
        }

        uint32_t count0 = st.idleCount0;
        uint32_t count1 = st.idleCount1;
        st.idleCount0 = 0;
        st.idleCount1 = 0;
        st.lastSampleTime = now;

        uint32_t rate0 = (uint32_t)(((uint64_t)count0 * 1000) / elapsed);
        uint32_t rate1 = (uint32_t)(((uint64_t)count1 * 1000) / elapsed);

        if (rate0 > st.maxIdleRate0) st.maxIdleRate0 = rate0;
        if (rate1 > st.maxIdleRate1) st.maxIdleRate1 = rate1;
        st.idleRate0 = rate0;
        st.idleRate1 = rate1;

        if (st.maxIdleRate0 > 1000) {
            float idlePct0 = ((float)rate0 / (float)st.maxIdleRate0) * 100.0f;
            float raw0 = _clamp(100.0f - idlePct0, 0.0f, 100.0f);
            st.cpu0Usage = (st.cpu0Usage * 0.4f) + (raw0 * 0.6f);
        }

        if (st.maxIdleRate1 > 1000) {
            float idlePct1 = ((float)rate1 / (float)st.maxIdleRate1) * 100.0f;
            float raw1 = _clamp(100.0f - idlePct1, 0.0f, 100.0f);
            st.cpu1Usage = (st.cpu1Usage * 0.4f) + (raw1 * 0.6f);
        }
        portEXIT_CRITICAL(&st.mux);
    }
};

// === src/ota_manager.h ===


class OTAManager {
public:
    static inline bool s_initialized{false};
    static inline bool s_otaStartedOk{false};
    static inline bool s_isFlashing{false};
    static inline size_t s_bytesWritten{0};
    static inline AsyncWebServerRequest* s_activeReq{nullptr};
    static inline portMUX_TYPE s_otaMux = portMUX_INITIALIZER_UNLOCKED;

    static void updatePassword(const char* password) {
        if (s_initialized && password && strlen(password) > 0) {
            ArduinoOTA.setPassword(password);
        }
    }

    static void beginArduinoOTA(const char* password = "admin") {
        if (s_initialized) return;
        // Never open unauthenticated ArduinoOTA, and do not listen in AP-only mode
        if (WiFi.status() != WL_CONNECTED) {
            return;
        }

        ArduinoOTA.setHostname("esp32-s3-dsp");
        ArduinoOTA.setPort(3232);
        ArduinoOTA.setMdnsEnabled(false);
        if (password && strlen(password) > 0) {
            ArduinoOTA.setPassword(password);
        }

        ArduinoOTA.onStart([]() {
            s_isFlashing = true;
            lms_pause_for_ota();
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Console.println("[OTA] ArduinoOTA update starting: " + type);
        });

        ArduinoOTA.onEnd([]() {
            s_isFlashing = false;
            Console.println("\n[OTA] ArduinoOTA update complete! Rebooting...");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            esp_task_wdt_reset();
            static unsigned int lastPct = 999;
            unsigned int pct = (total > 0) ? (progress * 100ULL / total) : 0;
            if (pct != lastPct && pct % 10 == 0) {
                lastPct = pct;
                Console.printf("[OTA] Progress: %u%%\n", pct);
            }
        });

        ArduinoOTA.onError([](ota_error_t error) {
            s_isFlashing = false;
            Console.printf("[OTA] ArduinoOTA Error[%u]\n", error);
        });

        ArduinoOTA.begin();
        s_initialized = true;
        Console.println("[OTA] ArduinoOTA service listening on port 3232 (Hostname: esp32-s3-dsp)");
    }

    static void loopArduinoOTA(const char* password = "admin") {
        if (s_initialized) {
            ArduinoOTA.handle();
        } else if (WiFi.status() == WL_CONNECTED) {
            beginArduinoOTA(password);
        }
    }

    static void setupWebRoutes(AsyncWebServer& server, ParamStore& params) {
        // GET /api/ota/auth - Pre-flight auth check so browser prompts before streaming firmware binary
        server.on("/api/ota/auth", HTTP_GET, [&params](AsyncWebServerRequest* req) {
            char u[32], p[64];
            params.getAuth(u, sizeof(u), p, sizeof(p));
            if (!req->authenticate(u, p)) {
                return req->requestAuthentication("ESP32-S3-DSP");
            }
            req->send(200, "application/json", "{\"ok\":true}");
        });

        // GET /update - Redirect legacy direct link to main UI OTA modal
        server.on("/update", HTTP_GET, [](AsyncWebServerRequest* req) {
            req->redirect("/?ota=1");
        });

        // POST /update - Handles upload stream and calls Update.write
        server.on(
            "/update",
            HTTP_POST,
            [&params](AsyncWebServerRequest* req) {
                char u[32], p[64];
                params.getAuth(u, sizeof(u), p, sizeof(p));
                if (!req->authenticate(u, p)) {
                    return req->requestAuthentication("ESP32-S3-DSP");
                }
                if (req->hasHeader("Origin")) {
                    String origin = req->header("Origin");
                    String host = req->host();
                    String expectedHttp = "http://" + host;
                    String expectedHttps = "https://" + host;
                    if (!origin.equalsIgnoreCase(expectedHttp) && !origin.equalsIgnoreCase(expectedHttps)) {
                        return req->send(403, "text/plain", "Forbidden: Cross-origin OTA rejected");
                    }
                }

                portENTER_CRITICAL(&s_otaMux);
                bool isOwner = (s_activeReq == req);
                bool ok = isOwner && s_otaStartedOk && !Update.hasError();
                if (isOwner) {
                    s_activeReq = nullptr;
                    s_isFlashing = false;
                    s_otaStartedOk = false;
                    s_bytesWritten = 0;
                }
                portEXIT_CRITICAL(&s_otaMux);

                if (!isOwner && s_isFlashing) {
                    return req->send(409, "text/plain", "OTA already in progress");
                }

                AsyncWebServerResponse* res = req->beginResponse(
                    ok ? 200 : 500,
                    "text/plain",
                    ok ? "OK" : "FAIL"
                );
                res->addHeader("Connection", "close");
                req->send(res);
                if (ok) {
                    Console.println("[OTA] Firmware flashed successfully! Flushing NVS and restarting...");
                    xTaskCreate([](void* arg) {
                        ParamStore* ps = static_cast<ParamStore*>(arg);
                        if (ps && ps->isDirty()) {
                            ps->save();
                        }
                        vTaskDelay(pdMS_TO_TICKS(600));
                        ESP.restart();
                    }, "OTAReboot", 4096, &params, 1, nullptr);
                } else if (isOwner) {
                    Update.abort();
                }
            },
            [&params](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final) {
                char u[32], p[64];
                params.getAuth(u, sizeof(u), p, sizeof(p));
                if (!req->authenticate(u, p)) {
                    return;
                }
                if (req->hasHeader("Origin")) {
                    String origin = req->header("Origin");
                    String host = req->host();
                    String expectedHttp = "http://" + host;
                    String expectedHttps = "https://" + host;
                    if (!origin.equalsIgnoreCase(expectedHttp) && !origin.equalsIgnoreCase(expectedHttps)) {
                        return;
                    }
                }

                const esp_partition_t* nextPart = esp_ota_get_next_update_partition(nullptr);
                const size_t maxPartitionSize = nextPart ? nextPart->size : 0x640000;

                if (!index) {
                    portENTER_CRITICAL(&s_otaMux);
                    if (s_isFlashing && s_activeReq != req) {
                        portEXIT_CRITICAL(&s_otaMux);
                        Console.println("[OTA] Rejected concurrent OTA upload attempt");
                        return;
                    }
                    s_activeReq = req;
                    s_isFlashing = true;
                    s_otaStartedOk = false;
                    s_bytesWritten = 0;
                    portEXIT_CRITICAL(&s_otaMux);

                    req->onDisconnect([req]() {
                        portENTER_CRITICAL(&s_otaMux);
                        bool wasActive = (s_activeReq == req && s_isFlashing);
                        if (wasActive) {
                            s_activeReq = nullptr;
                            s_isFlashing = false;
                            s_otaStartedOk = false;
                            s_bytesWritten = 0;
                        }
                        portEXIT_CRITICAL(&s_otaMux);
                        if (wasActive) {
                            Update.abort();
                            Console.println("[OTA] Client disconnected mid-upload; Update aborted.");
                        }
                    });

                    Console.printf("[OTA] Web upload started: %s\n", filename.c_str());

                    if (len < 1 || data[0] != 0xE9) {
                        Console.println("[OTA] Error: Invalid firmware image (missing ESP32 0xE9 magic byte)");
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        if (s_activeReq == req) { s_isFlashing = false; s_activeReq = nullptr; }
                        portEXIT_CRITICAL(&s_otaMux);
                        return;
                    }

                    size_t contentLength = req->contentLength();
                    if (contentLength > maxPartitionSize + 4096) {
                        Console.printf("[OTA] Error: Upload size (%u) exceeds partition size (%u)\n",
                                       (unsigned)contentLength, (unsigned)maxPartitionSize);
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        if (s_activeReq == req) { s_isFlashing = false; s_activeReq = nullptr; }
                        portEXIT_CRITICAL(&s_otaMux);
                        return;
                    }

                    lms_pause_for_ota();

                    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                        Console.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        if (s_activeReq == req) { s_isFlashing = false; s_activeReq = nullptr; }
                        portEXIT_CRITICAL(&s_otaMux);
                        return;
                    }
                    portENTER_CRITICAL(&s_otaMux);
                    if (s_activeReq == req) s_otaStartedOk = true;
                    portEXIT_CRITICAL(&s_otaMux);
                }

                portENTER_CRITICAL(&s_otaMux);
                bool activeOwner = (s_activeReq == req && s_otaStartedOk);
                portEXIT_CRITICAL(&s_otaMux);
                if (!activeOwner) return;

                if (len > 0 && !Update.hasError()) {
                    if (s_bytesWritten + len > maxPartitionSize) {
                        Console.printf("[OTA] Error: Firmware stream exceeded partition size (%u)\n", (unsigned)maxPartitionSize);
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        s_otaStartedOk = false;
                        s_isFlashing = false;
                        portEXIT_CRITICAL(&s_otaMux);
                        return;
                    }
                    if (Update.write(data, len) != len) {
                        Console.printf("[OTA] Update.write failed: %s\n", Update.errorString());
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        s_otaStartedOk = false;
                        s_isFlashing = false;
                        portEXIT_CRITICAL(&s_otaMux);
                    } else {
                        s_bytesWritten += len;
                    }
                }

                if (final) {
                    portENTER_CRITICAL(&s_otaMux);
                    bool stillOk = (s_activeReq == req && s_otaStartedOk && !Update.hasError());
                    portEXIT_CRITICAL(&s_otaMux);
                    if (stillOk && Update.end(true)) {
                        Console.printf("[OTA] Web upload finished: %u bytes written\n", (unsigned)s_bytesWritten);
                    } else {
                        Console.printf("[OTA] Update.end failed: %s\n", Update.errorString());
                        Update.abort();
                        portENTER_CRITICAL(&s_otaMux);
                        s_otaStartedOk = false;
                        portEXIT_CRITICAL(&s_otaMux);
                    }
                    portENTER_CRITICAL(&s_otaMux);
                    if (s_activeReq == req) s_isFlashing = false;
                    portEXIT_CRITICAL(&s_otaMux);
                }
            }
        );
    }
};

