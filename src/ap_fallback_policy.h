#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <cstring>

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
        // Handle probe URLs that carry query strings (e.g. /generate_204?...)
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

    // Only respond to standard queries (QR=0, Opcode=0) with at least one question.
    if ((req[2] & 0x80U) != 0 || (req[2] & 0x78U) != 0) {
        return 0;
    }
    const uint16_t qdCount = (static_cast<uint16_t>(req[4]) << 8) | req[5];
    if (qdCount < 1U) {
        return 0;
    }

    // Bounds-check Question #1 QName labels and optional compression pointer.
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

    // Resolve local DSP hostnames (IN/ANY A -> AP IP, AAAA/other -> NoError NODATA),
    // and return NXDOMAIN for external domains so phones don't open captive popups.
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

    // Ensure driver stays initialized while clearing STA credentials so
    // wifiLowLevelDeinit() -> esp_wifi_init() doesn't reload stale NVS STA config.
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

    // Guard against WiFiGeneric's static first_connect flipping mode back to AP_STA
    WiFi.setAutoReconnect(false);
    if (WiFi.getMode() != WIFI_AP) {
        if (WiFi.getMode() & WIFI_MODE_STA) {
            esp_wifi_disconnect();
        }
        WiFi.mode(WIFI_AP);
    }

    // Keep ESP-IDF's native 192.168.4.1/24 AP config & 100-address DHCPS pool intact.
    WiFi.softAP(apSsid, nullptr, 1, 0, 4, false);
    apEnsureSoftApDhcpRunning();
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_inactive_time(WIFI_IF_AP, 60);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
}
