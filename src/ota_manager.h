#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include "wireless_console.h"

class OTAManager {
public:
    static inline bool s_initialized{false};

    static void beginArduinoOTA() {
        if (s_initialized) return;
        if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
            return;
        }

        ArduinoOTA.setHostname("esp32-s3-dsp");
        ArduinoOTA.setPort(3232);
        ArduinoOTA.setMdnsEnabled(false);

        ArduinoOTA.onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Console.println("[OTA] ArduinoOTA update starting: " + type);
        });

        ArduinoOTA.onEnd([]() {
            Console.println("\n[OTA] ArduinoOTA update complete! Rebooting...");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            static unsigned int lastPct = 999;
            unsigned int pct = (progress / (total / 100));
            if (pct != lastPct && pct % 10 == 0) {
                lastPct = pct;
                Console.printf("[OTA] Progress: %u%%\n", pct);
            }
        });

        ArduinoOTA.onError([](ota_error_t error) {
            Console.printf("[OTA] ArduinoOTA Error[%u]\n", error);
        });

        ArduinoOTA.begin();
        s_initialized = true;
        Console.println("[OTA] ArduinoOTA service listening on port 3232 (Hostname: esp32-s3-dsp)");
    }

    static void loopArduinoOTA() {
        if (s_initialized) {
            ArduinoOTA.handle();
        } else if (WiFi.status() == WL_CONNECTED || WiFi.getMode() == WIFI_AP) {
            beginArduinoOTA();
        }
    }

    static void setupWebRoutes(AsyncWebServer& server) {
        // GET /update - Web-based firmware upload page
        server.on("/update", HTTP_GET, [](AsyncWebServerRequest* req) {
            const char* html = R"rawhtml(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-S3-DSP Wireless Firmware Update</title>
<style>
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #121214; color: #e1e1e6; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 90vh; }
.card { background: #1f1f24; border: 1px solid #323238; border-radius: 12px; padding: 30px; width: 100%; max-width: 480px; box-shadow: 0 8px 24px rgba(0,0,0,0.5); }
h2 { margin-top: 0; color: #a855f7; display: flex; align-items: center; gap: 10px; font-size: 20px; }
p { color: #a1a1aa; font-size: 14px; line-height: 1.5; }
.drop-zone { border: 2px dashed #4b5563; border-radius: 8px; padding: 25px; text-align: center; cursor: pointer; transition: 0.2s; background: #18181b; }
.drop-zone:hover { border-color: #a855f7; }
input[type="file"] { display: none; }
button { width: 100%; margin-top: 20px; background: #a855f7; color: #fff; border: none; padding: 12px; font-weight: 600; font-size: 15px; border-radius: 8px; cursor: pointer; transition: 0.2s; }
button:hover { background: #9333ea; }
button:disabled { background: #4b5563; cursor: not-allowed; }
.bar-wrap { background: #27272a; border-radius: 6px; height: 12px; overflow: hidden; margin-top: 20px; display: none; }
.bar { width: 0%; height: 100%; background: #a855f7; transition: width 0.1s; }
.status { margin-top: 15px; font-size: 14px; text-align: center; color: #e4e4e7; min-height: 20px; }
.nav-link { display: inline-block; margin-top: 15px; color: #a855f7; text-decoration: none; font-size: 13px; }
.nav-link:hover { text-decoration: underline; }
</style>
</head>
<body>
<div class="card">
  <h2><svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg> Firmware Update (OTA)</h2>
  <p>Select or drag and drop <code>firmware.bin</code> compiled by PlatformIO.</p>
  
  <div class="drop-zone" id="dropZone" onclick="document.getElementById('fileInput').click()">
    <div id="dropText">Click to choose firmware.bin or drop file here</div>
    <input type="file" id="fileInput" accept=".bin" onchange="fileSelected(this)">
  </div>

  <div class="bar-wrap" id="barWrap">
    <div class="bar" id="bar"></div>
  </div>

  <div class="status" id="status"></div>
  <button id="upBtn" disabled onclick="uploadFirmware()">Upload Firmware</button>
  <div style="text-align: center;">
    <a href="/" class="nav-link">&larr; Back to DSP Dashboard</a> &bull;
    <a href="/console" class="nav-link">Live Console &rarr;</a>
  </div>
</div>

<script>
let selFile = null;
function fileSelected(input) {
  if (input.files && input.files[0]) {
    selFile = input.files[0];
    document.getElementById('dropText').innerText = selFile.name + " (" + (selFile.size/1024).toFixed(1) + " KB)";
    document.getElementById('upBtn').disabled = false;
  }
}

const dropZone = document.getElementById('dropZone');
dropZone.addEventListener('dragover', (e) => { e.preventDefault(); dropZone.style.borderColor = '#a855f7'; });
dropZone.addEventListener('dragleave', (e) => { e.preventDefault(); dropZone.style.borderColor = '#4b5563'; });
dropZone.addEventListener('drop', (e) => {
  e.preventDefault();
  dropZone.style.borderColor = '#4b5563';
  if (e.dataTransfer.files && e.dataTransfer.files[0]) {
    selFile = e.dataTransfer.files[0];
    document.getElementById('dropText').innerText = selFile.name + " (" + (selFile.size/1024).toFixed(1) + " KB)";
    document.getElementById('upBtn').disabled = false;
  }
});

function uploadFirmware() {
  if (!selFile) return;
  const upBtn = document.getElementById('upBtn');
  const barWrap = document.getElementById('barWrap');
  const bar = document.getElementById('bar');
  const status = document.getElementById('status');

  upBtn.disabled = true;
  barWrap.style.display = 'block';
  status.innerText = 'Uploading firmware...';

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update', true);

  xhr.upload.onprogress = (e) => {
    if (e.lengthComputable) {
      const pct = Math.round((e.loaded / e.total) * 100);
      bar.style.width = pct + '%';
      status.innerText = 'Uploading: ' + pct + '% (' + (e.loaded/1024).toFixed(0) + ' / ' + (e.total/1024).toFixed(0) + ' KB)';
    }
  };

  xhr.onload = () => {
    if (xhr.status === 200) {
      status.style.color = '#4ade80';
      status.innerText = 'Upload successful! ESP32-S3 is rebooting...';
      bar.style.width = '100%';
      let countdown = 10;
      const interval = setInterval(() => {
        countdown--;
        if (countdown <= 0) {
          clearInterval(interval);
          window.location.href = '/';
        } else {
          status.innerText = 'Rebooting... redirecting in ' + countdown + 's';
        }
      }, 1000);
    } else {
      status.style.color = '#ef4444';
      status.innerText = 'Upload failed: ' + xhr.responseText;
      upBtn.disabled = false;
    }
  };

  xhr.onerror = () => {
    status.style.color = '#ef4444';
    status.innerText = 'Network error during firmware upload.';
    upBtn.disabled = false;
  };

  const formData = new FormData();
  formData.append('firmware', selFile, selFile.name);
  xhr.send(formData);
}
</script>
</body>
</html>)rawhtml";
            req->send(200, "text/html", html);
        });

        // POST /update - Handles upload stream and calls Update.write
        server.on(
            "/update",
            HTTP_POST,
            [](AsyncWebServerRequest* req) {
                bool ok = !Update.hasError();
                AsyncWebServerResponse* res = req->beginResponse(
                    ok ? 200 : 500,
                    "text/plain",
                    ok ? "OK" : "FAIL"
                );
                res->addHeader("Connection", "close");
                req->send(res);
                if (ok) {
                    Console.println("[OTA] Firmware flashed successfully! Restarting in 500ms...");
                    xTaskCreate([](void*) {
                        vTaskDelay(pdMS_TO_TICKS(600));
                        ESP.restart();
                    }, "OTAReboot", 2048, nullptr, 1, nullptr);
                }
            },
            [](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final) {
                if (!index) {
                    Console.printf("[OTA] Web upload started: %s\n", filename.c_str());
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                        Console.printf("[OTA] Update.begin failed: %s\n", Update.errorString());
                    }
                }
                if (!Update.hasError()) {
                    if (Update.write(data, len) != len) {
                        Console.printf("[OTA] Update.write failed: %s\n", Update.errorString());
                    }
                }
                if (final) {
                    if (Update.end(true)) {
                        Console.printf("[OTA] Web upload finished: %u bytes\n", index + len);
                    } else {
                        Console.printf("[OTA] Update.end failed: %s\n", Update.errorString());
                    }
                }
            }
        );
    }
};

