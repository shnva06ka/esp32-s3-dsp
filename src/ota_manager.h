#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "wireless_console.h"
#include "param_store.h"
#include "squeezelite.h"

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
            slimproto_pause_for_ota();
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Console.println("[OTA] ArduinoOTA update starting: " + type);
        });

        ArduinoOTA.onEnd([]() {
            s_isFlashing = false;
            Console.println("\n[OTA] ArduinoOTA update complete! Rebooting...");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
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

                    slimproto_pause_for_ota();

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

