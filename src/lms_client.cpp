#include "lms_client.h"

#include <WiFiUdp.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <string.h>

#include "audio_config.h"
#include "audio_ring_buffer.h"
#include "i2s_audio.h"
#include "lms_core.h"
#include "lms_codecs.h"
#include "wireless_console.h"

namespace {

constexpr const char* kTag = "[LMS]";

constexpr uint16_t kServerPort      = 3483;
constexpr size_t   kStreamRingBytes = 2u * 1024u * 1024u;   // power of two, PSRAM
constexpr size_t   kOutRingFrames   = 262144;               // 2 MiB planar float in PSRAM (~1.37 s at 192 kHz)
constexpr size_t   kChunkFrames     = RS_MAX_IN;            // decoder chunk, also the resampler's max input
constexpr size_t   kResampleOutCap  = 4096;
constexpr size_t   kHttpHeaderMax   = 4096;
constexpr size_t   kHttpReqMax      = 2048;
constexpr uint32_t kYieldEveryMs    = 15;
constexpr uint32_t kTickMs          = 100;
constexpr uint32_t kHeartbeatMs     = 1000;
constexpr uint32_t kAbortWaitMs     = 3000;

struct TrackParams {
    uint32_t ip{0};                 // 0 = control server
    uint16_t port{9000};
    uint8_t  http[kHttpReqMax];
    size_t   httpLen{0};
    char     format{'f'};
    char     pcmSize{'?'};
    char     pcmRate{'?'};
    char     pcmChannels{'?'};
    char     pcmEndian{'?'};
    uint32_t replayGain{0};
    uint8_t  flags{0};
    bool     gaplessBoundary{false};
};

struct Core {
    // lifecycle
    std::atomic<bool> running{false};
    std::atomic<bool> otaPause{false};
    std::atomic<bool> controlIdle{true};
    TaskHandle_t      controlTask{nullptr}, streamTask{nullptr}, decodeTask{nullptr};
    std::atomic<bool> controlExited{true}, streamExited{true}, decodeExited{true};

    // buffers
    uint8_t*          streamStore{nullptr};
    lms::ByteRing     ring;
    AudioRingBuffer<kOutRingFrames> out;
    SemaphoreHandle_t outMutex{nullptr};
    float*            inL{nullptr};
    float*            inR{nullptr};
    float*            rsL{nullptr};
    float*            rsR{nullptr};
    int32_t*          s32{nullptr};
    uint8_t*          pcmBuf{nullptr};
    uint8_t*          ctlBuf{nullptr};
    uint8_t*          tmpBuf{nullptr};

    // track session handshake: tasks only touch ring/params for the generation named in readyGen
    std::atomic<uint32_t> gen{0}, readyGen{0}, streamAck{0}, decodeAck{0};
    TrackParams       tp;
    IPAddress         controlServer;

    // session flags (written by the stream/decode tasks, consumed by the control task)
    std::atomic<bool> streamEof{false}, streamError{false}, evConnected{false}, evHeaders{false};
    std::atomic<bool> decoderDone{false}, decoderError{false};
    std::atomic<uint32_t> bytesRx{0};
    std::atomic<uint32_t> srcRate{0}, srcBits{0};

    // output side
    std::atomic<bool>     playing{false}, outPaused{false}, dacEnabled{true};
    std::atomic<uint32_t> audgL{0x10000u}, audgR{0x10000u};
    std::atomic<float>    volL{1.0f}, volR{1.0f};
    std::atomic<uint32_t> framesWritten{0}, framesRead{0}, trackFrames{0}, skipFrames{0}, boundaryAt{0};
    std::atomic<bool>     boundaryPending{false}, boundaryHit{false}, underrun{false};
    float                 curL{1.0f}, curR{1.0f};     // audio-task private

    // telemetry
    ParamStore*           params{nullptr};
    std::atomic<uint32_t> activeOutRate{audio_rate::kDefaultRate};
    std::atomic<bool>     connected{false};
    std::atomic<uint32_t> serverIpRaw{0};
    std::atomic<uint8_t>  phase{(uint8_t)lms::Phase::Off};
    SemaphoreHandle_t     rsMutex{nullptr};
    resampler_t*          rs{nullptr};

    // skip/glitch diagnostics (atomics updated in hot paths, reported in telemetry)
    std::atomic<uint32_t> diagAudioCalls{0};
    std::atomic<uint32_t> diagAudioShort{0};
    std::atomic<uint32_t> diagAudioShortFrames{0};
    std::atomic<uint32_t> diagMutexMiss{0};
    std::atomic<uint32_t> diagMaxAudioUs{0};
    std::atomic<uint32_t> diagMaxDecodeMs{0};
    std::atomic<uint32_t> diagMaxRingWaitMs{0};
    std::atomic<uint32_t> diagMaxOutWaitMs{0};
    std::atomic<uint32_t> diagMinRingBytes{0xFFFFFFFFu};
    std::atomic<uint32_t> diagMinOutFrames{0xFFFFFFFFu};
    std::atomic<uint32_t> diagRebuffers{0};
    uint32_t              lastAudioUs{0};
};

Core C;

uint32_t nowMs() { return (uint32_t)millis(); }

void* allocPsramFirst(size_t bytes) {
    void* p = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return p;
}

// ------------------------------------------------------------------ output side

bool flushOutput() {
    if (!C.outMutex) return false;
    xSemaphoreTake(C.outMutex, portMAX_DELAY);
    C.out.skip(C.out.available());
    C.framesRead.store(C.framesWritten.load());
    C.trackFrames.store(0);
    C.skipFrames.store(0);
    C.boundaryPending.store(false);
    C.boundaryHit.store(false);
    C.underrun.store(false);
    xSemaphoreGive(C.outMutex);
    return true;
}

// ------------------------------------------------------------------ session handshake

bool waitForAcks(uint32_t gen) {
    const uint32_t t0 = nowMs();
    while (C.streamAck.load(std::memory_order_acquire) != gen ||
           C.decodeAck.load(std::memory_order_acquire) != gen) {
        if (nowMs() - t0 > kAbortWaitMs) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

void resetSessionFlags() {
    C.streamEof.store(false);
    C.streamError.store(false);
    C.evConnected.store(false);
    C.evHeaders.store(false);
    C.decoderDone.store(false);
    C.decoderError.store(false);
    C.srcRate.store(0);
    C.srcBits.store(0);
}

// Stops the stream and decode tasks for the current track. Returns false if they failed to stop in time.
bool abortSession() {
    const uint32_t g = C.gen.fetch_add(1, std::memory_order_acq_rel) + 1;
    if (!waitForAcks(g)) return false;
    C.ring.reset();
    resetSessionFlags();
    return true;
}

bool startSession(const lms::Strm& s, bool gapless) {
    if (!abortSession()) return false;
    if (!gapless) flushOutput();

    TrackParams& t = C.tp;
    t.ip = s.ip;
    t.port = s.port ? s.port : 9000;
    t.httpLen = s.httpLen < kHttpReqMax ? s.httpLen : kHttpReqMax;
    memcpy(t.http, s.http, t.httpLen);
    t.format = s.format;
    t.pcmSize = s.pcmSize;
    t.pcmRate = s.pcmRate;
    t.pcmChannels = s.pcmChannels;
    t.pcmEndian = s.pcmEndian;
    t.replayGain = s.replayGain;
    t.flags = s.flags;
    t.gaplessBoundary = gapless;

    C.readyGen.store(C.gen.load(), std::memory_order_release);
    return true;
}

// ------------------------------------------------------------------ stream task (HTTP -> byte ring)

bool readHttpHeaders(WiFiClient& cl, uint32_t gen) {
    char    head[kHttpHeaderMax];
    size_t  n = 0;
    const uint32_t t0 = nowMs();
    while (n < sizeof(head) - 1) {
        if (C.gen.load() != gen) return false;
        if (nowMs() - t0 > 8000) return false;
        if (!cl.available()) {
            if (!cl.connected()) return false;
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        const int c = cl.read();
        if (c < 0) continue;
        head[n++] = (char)c;
        if (n >= 4 && memcmp(head + n - 4, "\r\n\r\n", 4) == 0) {
            head[n] = 0;
            return strncmp(head, "HTTP/1.", 7) == 0 && strstr(head, " 200") != nullptr &&
                   (size_t)(strstr(head, " 200") - head) < 12;
        }
    }
    return false;
}

void streamTaskFn(void*) {
    WiFiClient client;
    uint32_t active = 0, lastServed = 0;
    uint32_t lastYield = nowMs();
    uint32_t lastData = 0;
    bool     headersDone = false;

    while (C.running.load(std::memory_order_acquire)) {
        const uint32_t g = C.gen.load(std::memory_order_acquire);
        if (active != 0 && g != active) {
            client.stop();
            active = 0;
        }
        if (active == 0) {
            C.streamAck.store(g, std::memory_order_release);
            if (g != 0 && g != lastServed && C.readyGen.load(std::memory_order_acquire) == g) {
                active = g;
                lastServed = g;
                headersDone = false;
                lastData = nowMs();
                IPAddress ip = C.tp.ip ? IPAddress((uint32_t)__builtin_bswap32(C.tp.ip)) : C.controlServer;
                if (!client.connect(ip, C.tp.port, 3000) ||
                    client.write(C.tp.http, C.tp.httpLen) != C.tp.httpLen) {
                    Console.printf("%s stream connect/request failed\n", kTag);
                    C.streamError.store(true);
                    client.stop();
                    active = 0;
                    continue;
                }
                client.setNoDelay(true);
                C.evConnected.store(true);
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
        }

        if (!headersDone) {
            if (!readHttpHeaders(client, active)) {
                if (C.gen.load() == active) {
                    Console.printf("%s HTTP header error\n", kTag);
                    C.streamError.store(true);
                }
                client.stop();
                active = 0;
                continue;
            }
            headersDone = true;
            C.evHeaders.store(true);
        }

        size_t room = C.ring.space();
        if (room > 0) {
            const int avail = client.available();
            if (avail > 0) {
                size_t want = (size_t)avail < room ? (size_t)avail : room;
                if (want > 4096) want = 4096;
                const int r = client.read(C.tmpBuf, want);
                if (r > 0) {
                    C.ring.write(C.tmpBuf, (size_t)r);
                    C.bytesRx.fetch_add((uint32_t)r);
                    lastData = nowMs();
                }
            } else if (!client.connected()) {
                C.streamEof.store(true);
                client.stop();
                active = 0;
                continue;
            } else if (nowMs() - lastData > 20000) {
                Console.printf("%s stream stalled\n", kTag);
                C.streamError.store(true);
                client.stop();
                active = 0;
                continue;
            } else {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (nowMs() - lastYield >= 10) {
            vTaskDelay(pdMS_TO_TICKS(2));
            lastYield = nowMs();
        }
    }
    client.stop();
    C.streamExited.store(true);
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------------ decode task (byte ring -> float ring)

struct ReadCtx {
    uint32_t gen;
};

// Blocking read from the byte ring. Returns 0 on end of stream, abort or stall.
size_t ringRead(void* user, void* buf, size_t n) {
    const uint32_t gen = static_cast<ReadCtx*>(user)->gen;
    uint32_t waited = 0;
    for (;;) {
        const size_t got = C.ring.read((uint8_t*)buf, n);
        if (got > 0) {
            if (waited > C.diagMaxRingWaitMs.load(std::memory_order_relaxed)) {
                C.diagMaxRingWaitMs.store(waited, std::memory_order_relaxed);
            }
            if (!C.streamEof.load(std::memory_order_relaxed)) {
                const uint32_t av = (uint32_t)C.ring.available();
                uint32_t prev = C.diagMinRingBytes.load(std::memory_order_relaxed);
                while (av < prev && !C.diagMinRingBytes.compare_exchange_weak(prev, av, std::memory_order_relaxed)) {}
            }
            return got;
        }
        if (C.gen.load(std::memory_order_acquire) != gen) return 0;
        if (C.streamEof.load(std::memory_order_acquire) && C.ring.available() == 0) return 0;
        if (C.streamError.load(std::memory_order_acquire)) return 0;
        if (waited > 15000) return 0;
        vTaskDelay(pdMS_TO_TICKS(2));
        waited += 2;
    }
}

// Blocks until `n` bytes are read, or the stream ends (returns the partial count).
size_t ringReadFull(ReadCtx* ctx, uint8_t* buf, size_t n) {
    size_t total = 0;
    while (total < n) {
        const size_t got = ringRead(ctx, buf + total, n - total);
        if (got == 0) break;
        total += got;
    }
    return total;
}

bool prepareResampler(uint32_t rate) {
    if (C.params) C.params->setWifiInputRate(rate);
    const uint32_t maxDac = C.params ? C.params->getMaxDacRate() : audio_rate::kDefaultRate;
    const uint32_t targetRate = audio_rate::activeRate(
        audio_rate::Source::Wifi, audio_rate::kDefaultRate, maxDac, rate);

    i2s_request_wifi_rate(targetRate);
    if (targetRate != i2s_active_rate()) {
        // Drain barrier: let any old-rate frames in the output ring finish before switching clocks.
        const uint32_t genSnap = C.gen.load(std::memory_order_acquire);
        uint32_t waited = 0;
        while (C.playing.load(std::memory_order_acquire) &&
               C.out.available() > 0 &&
               C.gen.load(std::memory_order_acquire) == genSnap &&
               C.running.load(std::memory_order_acquire) &&
               waited < 2000) {
            vTaskDelay(pdMS_TO_TICKS(5));
            waited += 5;
        }
        if (C.params && C.params->getInputMode() == INPUT_MODE_WIFI) {
            for (uint32_t w = 0; w < 200 && i2s_active_rate() != targetRate; w += 2) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
    }

    C.activeOutRate.store(targetRate, std::memory_order_release);
    Console.printf("%s stream %u Hz -> target %u Hz (maxDac=%u)\n",
                   kTag, (unsigned)rate, (unsigned)targetRate, (unsigned)maxDac);

    xSemaphoreTake(C.rsMutex, portMAX_DELAY);
    if (C.rs) {
        resampler_destroy(C.rs);
        C.rs = nullptr;
    }
    C.rs = resampler_create(rate, targetRate);
    xSemaphoreGive(C.rsMutex);
    return C.rs != nullptr;
}

// Writes planar float frames to the output ring, waiting for room. Returns false when aborted.
bool writeOutput(uint32_t gen, const float* L, const float* R, size_t n) {
    uint32_t waited = 0;
    while (n > 0) {
        if (C.gen.load(std::memory_order_acquire) != gen) return false;
        const size_t w = C.out.write(L, R, n);
        if (w == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            waited += 5;
            continue;
        }
        C.framesWritten.fetch_add((uint32_t)w);
        L += w;
        R += w;
        n -= w;
    }
    if (waited > C.diagMaxOutWaitMs.load(std::memory_order_relaxed)) {
        C.diagMaxOutWaitMs.store(waited, std::memory_order_relaxed);
    }
    return true;
}

// Resamples a block of planar float input and pushes it to the output ring.
bool emitBlock(uint32_t gen, float* L, float* R, size_t n, const lms::Gains& tg) {
    if (tg.left != 1.0f || tg.right != 1.0f) {
        for (size_t i = 0; i < n; ++i) {
            L[i] *= tg.left;
            R[i] *= tg.right;
        }
    }
    if (resampler_is_passthrough(C.rs)) return writeOutput(gen, L, R, n);
    const size_t produced = resampler_process(C.rs, L, R, n, C.rsL, C.rsR, kResampleOutCap);
    return produced == 0 ? true : writeOutput(gen, C.rsL, C.rsR, produced);
}

void decodeFlac(uint32_t gen, ReadCtx* ctx, const lms::Gains& tg) {
    unsigned rate = 0, channels = 0, bits = 0;
    lms_flac_t* f = lms_flac_open(ringRead, ctx, &rate, &channels, &bits);
    if (!f) {
        if (C.gen.load() == gen && !C.streamError.load()) {
            Console.printf("%s FLAC open failed\n", kTag);
            C.decoderError.store(true);
        }
        return;
    }
    Console.printf("%s FLAC %u Hz, %u-bit, %u ch\n", kTag, rate, bits, channels);
    C.srcRate.store(rate);
    C.srcBits.store(bits);
    if (!prepareResampler(rate)) {
        Console.printf("%s resampler unavailable for %u Hz\n", kTag, rate);
        C.decoderError.store(true);
        lms_flac_close(f);
        return;
    }
    if (C.tp.gaplessBoundary) {
        C.boundaryAt.store(C.framesWritten.load());
        C.boundaryPending.store(true);
    }

    size_t chunk = kChunkFrames;
    if (!resampler_is_passthrough(C.rs)) {
        const size_t fit = resampler_in_for_out(C.rs, kResampleOutCap);
        if (fit < chunk) chunk = fit > 16 ? fit - 8 : 8;
    }
    uint32_t lastYield = nowMs();
    constexpr float kScale = 1.0f / 2147483648.0f;

    for (;;) {
        if (C.gen.load(std::memory_order_acquire) != gen) break;
        const uint32_t tDec0 = nowMs();
        const size_t n = lms_flac_read(f, C.s32, chunk);
        if (n == 0) break;
        if (channels == 2) {
            for (size_t i = 0; i < n; ++i) {
                C.inL[i] = (float)C.s32[2 * i] * kScale;
                C.inR[i] = (float)C.s32[2 * i + 1] * kScale;
            }
        } else {
            for (size_t i = 0; i < n; ++i) {
                C.inL[i] = C.inR[i] = (float)C.s32[i] * kScale;
            }
        }
        if (!emitBlock(gen, C.inL, C.inR, n, tg)) break;
        const uint32_t decMs = nowMs() - tDec0;
        if (decMs > C.diagMaxDecodeMs.load(std::memory_order_relaxed)) {
            C.diagMaxDecodeMs.store(decMs, std::memory_order_relaxed);
        }
        if (nowMs() - lastYield >= 10) {
            vTaskDelay(pdMS_TO_TICKS(2));
            lastYield = nowMs();
        }
    }
    lms_flac_close(f);
}

void decodePcm(uint32_t gen, ReadCtx* ctx, const lms::Gains& tg) {
    const TrackParams& t = C.tp;
    lms::PcmInfo info;
    size_t have = 0;
    constexpr size_t kBuf = 8192;

    const uint32_t rawRate = lms::pcmRateFromCode(t.pcmRate);
    const unsigned rawBits = lms::pcmBitsFromCode(t.pcmSize);
    if (rawRate != 0 && rawBits != 0 && (t.pcmChannels == '1' || t.pcmChannels == '2')) {
        info.rate = rawRate;
        info.bits = (uint8_t)rawBits;
        info.channels = (uint8_t)(t.pcmChannels - '0');
        info.bigEndian = (t.pcmEndian == '0');
    } else {
        for (;;) {
            if (have >= 512) { have = 0; break; }
            const size_t got = ringRead(ctx, C.pcmBuf + have, 512 - have);
            if (got == 0) break;
            have += got;
            const int r = lms::parsePcmHeader(C.pcmBuf, have, info);
            if (r > 0) {
                memmove(C.pcmBuf, C.pcmBuf + info.headerBytes, have - info.headerBytes);
                have -= info.headerBytes;
                break;
            }
            if (r == lms::kPcmInvalid) { have = 512; }
        }
        if (info.rate == 0) {
            if (C.gen.load() == gen) {
                Console.printf("%s unsupported PCM stream\n", kTag);
                C.decoderError.store(true);
            }
            return;
        }
    }
    Console.printf("%s PCM %u Hz, %u-bit, %u ch\n", kTag, (unsigned)info.rate, info.bits, info.channels);
    C.srcRate.store(info.rate);
    C.srcBits.store(info.bits);
    if (!prepareResampler(info.rate)) {
        C.decoderError.store(true);
        return;
    }
    if (C.tp.gaplessBoundary) {
        C.boundaryAt.store(C.framesWritten.load());
        C.boundaryPending.store(true);
    }

    const size_t frameBytes = lms::pcmFrameBytes(info);
    size_t chunk = kChunkFrames;
    if (!resampler_is_passthrough(C.rs)) {
        const size_t fit = resampler_in_for_out(C.rs, kResampleOutCap);
        if (fit < chunk) chunk = fit > 16 ? fit - 8 : 8;
    }
    if (chunk * frameBytes > kBuf) chunk = kBuf / frameBytes;
    uint32_t lastYield = nowMs();

    for (;;) {
        if (C.gen.load(std::memory_order_acquire) != gen) break;
        const size_t want = chunk * frameBytes;
        if (have < want) have += ringReadFull(ctx, C.pcmBuf + have, want - have);
        const size_t frames = have / frameBytes;
        if (frames == 0) break;
        lms::pcmToFloat(info, C.pcmBuf, frames, C.inL, C.inR);
        const size_t used = frames * frameBytes;
        memmove(C.pcmBuf, C.pcmBuf + used, have - used);
        have -= used;
        if (!emitBlock(gen, C.inL, C.inR, frames, tg)) break;
        if (nowMs() - lastYield >= 10) {
            vTaskDelay(pdMS_TO_TICKS(2));
            lastYield = nowMs();
        }
        if (have < frameBytes && C.streamEof.load() && C.ring.available() == 0) break;
    }
}

void decodeTaskFn(void*) {
    uint32_t active = 0, lastServed = 0;
    while (C.running.load(std::memory_order_acquire)) {
        const uint32_t g = C.gen.load(std::memory_order_acquire);
        active = 0;
        C.decodeAck.store(g, std::memory_order_release);
        if (g == 0 || g == lastServed || C.readyGen.load(std::memory_order_acquire) != g) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        active = g;
        lastServed = g;

        ReadCtx ctx{g};
        const lms::Gains tg = lms::trackGains(C.tp.replayGain, C.tp.flags);
        if (C.tp.format == 'f') decodeFlac(g, &ctx, tg);
        else                    decodePcm(g, &ctx, tg);

        if (C.gen.load(std::memory_order_acquire) == g && !C.decoderError.load()) {
            C.decoderDone.store(true, std::memory_order_release);
        }
        // Idle until the control task starts another session; the ack at the loop top lets it proceed.
    }
    C.decodeExited.store(true);
    vTaskDelete(nullptr);
}

// ------------------------------------------------------------------ control task (SlimProto)

bool discoverServer(IPAddress& out) {
    WiFiUDP udp;
    if (!udp.begin(0)) return false;
    static const uint8_t kProbe[1] = {'e'};
    const IPAddress local = WiFi.localIP();
    const IPAddress mask  = WiFi.subnetMask();
    const IPAddress directed((uint32_t)local | ~(uint32_t)mask);
    for (int attempt = 0; attempt < 3 && C.running.load() && !C.otaPause.load(); ++attempt) {
        udp.beginPacket(IPAddress(255, 255, 255, 255), kServerPort);
        udp.write(kProbe, sizeof(kProbe));
        udp.endPacket();
        udp.beginPacket(directed, kServerPort);
        udp.write(kProbe, sizeof(kProbe));
        udp.endPacket();
        const uint32_t t0 = nowMs();
        while (nowMs() - t0 < 1000) {
            const int sz = udp.parsePacket();
            if (sz > 0) {
                const int first = udp.read();
                if (first == 'E') {
                    out = udp.remoteIP();
                    udp.stop();
                    return true;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    udp.stop();
    return false;
}

struct ControlState {
    lms::Transport transport;
    uint64_t       bytesTotal{0};
    uint32_t       bytesLast{0};
    uint32_t       lastServerTs{0};
    uint32_t       redirectIp{0};
    bool           reconnect{false};
};

void syncOutputFlags(ControlState& st) {
    C.playing.store(st.transport.isPlaying(), std::memory_order_release);
    C.outPaused.store(st.transport.isPaused(), std::memory_order_release);
    C.phase.store((uint8_t)st.transport.phase(), std::memory_order_release);
}

void sendStat(WiFiClient& cl, ControlState& st, uint32_t singleEvent) {
    lms::StatInfo s;
    s.streamBufSize = (uint32_t)kStreamRingBytes;
    s.streamBufFullness = (uint32_t)C.ring.available();
    s.bytesReceived = st.bytesTotal;
    const int rssi = WiFi.RSSI();
    int sig = 2 * (rssi + 100);
    s.signal = (uint16_t)(sig < 0 ? 0 : (sig > 100 ? 100 : sig));
    s.jiffies = nowMs();
    s.outBufSize = (uint32_t)(kOutRingFrames * 8);
    s.outBufFullness = (uint32_t)(C.out.available() * 8);
    const uint32_t tf = C.trackFrames.load();
    const uint32_t outRate = C.activeOutRate.load(std::memory_order_acquire);
    s.elapsedSeconds = outRate ? (tf / outRate) : 0;
    s.elapsedMs = outRate ? (uint32_t)(((uint64_t)tf * 1000ull) / outRate) : 0;
    s.serverTimestamp = st.lastServerTs;
    uint8_t buf[lms::kStatTotalLen];
    const size_t n = lms::buildStat(buf, sizeof(buf), lms::eventCode(singleEvent), s);
    if (n) cl.write(buf, n);
}

void sendEvents(WiFiClient& cl, ControlState& st, uint32_t events) {
    for (uint32_t bit = 1; bit <= lms::EV_t; bit <<= 1) {
        if (events & bit) sendStat(cl, st, bit);
    }
}

void stopPlayback(ControlState& st) {
    abortSession();
    flushOutput();
    st.transport.reset();
    syncOutputFlags(st);
}

uint32_t handleStrm(const lms::Frame& f, ControlState& st) {
    lms::Strm s;
    if (!lms::parseStrm(f.data, f.dataLen, s)) return 0;
    uint32_t ev = 0;
    switch (s.command) {
        case 's': {
            if (s.format != 'f' && s.format != 'p') {
                Console.printf("%s unsupported format '%c'\n", kTag, s.format);
                return lms::EV_c | lms::EV_n;
            }
            const bool gapless = (s.flags & lms::kFlagGapless) && st.transport.isPlaying();
            const uint32_t outRate = C.activeOutRate.load(std::memory_order_acquire);
            const uint32_t maxDac  = C.params ? C.params->getMaxDacRate() : outRate;
            const uint32_t thrRate = (outRate > maxDac) ? outRate : maxDac;
            const uint32_t tenthsCap = thrRate >= 10u ? (uint32_t)((kOutRingFrames / 2u) / (thrRate / 10u)) : 8u;
            uint8_t outThr = s.outputThreshold < 8 ? 8 : s.outputThreshold;
            if (outThr > tenthsCap && tenthsCap >= 1u) outThr = (uint8_t)tenthsCap;
            ev = st.transport.onStart(s.autostart, s.threshold, outThr, gapless, outRate);
            if (!startSession(s, gapless)) {
                Console.printf("%s could not stop previous track\n", kTag);
                ev |= st.transport.onStreamError();
            }
            break;
        }
        case 'p': ev = st.transport.onPause(s.replayGain, nowMs()); break;
        case 'u': ev = st.transport.onUnpause(); break;
        case 'q': stopPlayback(st); ev = lms::EV_f; break;
        case 'f': stopPlayback(st); ev = lms::EV_f; break;
        case 't': st.lastServerTs = s.replayGain; ev = st.transport.onStatusRequest(); break;
        case 'a': {
            const uint32_t outRate = C.activeOutRate.load(std::memory_order_acquire);
            C.skipFrames.fetch_add((uint32_t)(((uint64_t)s.replayGain * outRate) / 1000ull));
            break;
        }
        default: break;
    }
    return ev;
}

void handleFrame(const lms::Frame& f, ControlState& st, WiFiClient& cl) {
    uint32_t ev = 0;
    if (lms::opIs(f, "strm")) {
        ev = handleStrm(f, st);
    } else if (lms::opIs(f, "audg")) {
        lms::Audg a;
        if (lms::parseAudg(f.data, f.dataLen, a)) {
            C.audgL.store(a.left);
            C.audgR.store(a.right);
            const lms::Gains g = lms::volumeGains(a.left, a.right, C.dacEnabled.load());
            C.volL.store(g.left);
            C.volR.store(g.right);
        }
    } else if (lms::opIs(f, "aude")) {
        if (f.dataLen >= 2) {
            const bool dacOn = (f.data[1] != 0);
            C.dacEnabled.store(dacOn);
            const lms::Gains g = lms::volumeGains(C.audgL.load(), C.audgR.load(), dacOn);
            C.volL.store(g.left);
            C.volR.store(g.right);
        }
    } else if (lms::opIs(f, "serv")) {
        if (f.dataLen >= 4) {
            const uint32_t ip = lms::get32(f.data);
            if (ip > 1) {
                st.redirectIp = ip;
                st.reconnect = true;
            }
        }
    }
    // every other command (display, visualiser, firmware, ...) is ignored by length
    syncOutputFlags(st);
    if (ev) sendEvents(cl, st, ev);
}

void runConnection(const IPAddress& server, IPAddress& redirectOut, bool& hasRedirectOut) {
    hasRedirectOut = false;
    WiFiClient cl;
    if (!cl.connect(server, kServerPort, 3000)) {
        Console.printf("%s connect to %s failed\n", kTag, server.toString().c_str());
        return;
    }
    cl.setNoDelay(true);
    C.controlServer = server;
    Console.printf("%s connected to %s\n", kTag, server.toString().c_str());

    uint8_t mac[6];
    WiFi.macAddress(mac);
    const uint32_t heloMaxRate = C.params ? C.params->getMaxDacRate() : audio_rate::kDefaultRate;
    char caps[128];
    snprintf(caps, sizeof(caps),
             "flc,pcm,MaxSampleRate=%u,Model=squeezelite,ModelName=ESP32-S3-DSP,AccuratePlayPoints",
             (unsigned)heloMaxRate);
    uint8_t helo[192];
    const size_t hn = lms::buildHelo(helo, sizeof(helo), mac, 12, 0, 0, caps);
    if (!hn || cl.write(helo, hn) != hn) return;
    Console.printf("%s HELO MaxSampleRate=%u\n", kTag, (unsigned)heloMaxRate);

    C.connected.store(true);
    C.serverIpRaw.store((uint32_t)server);

    ControlState st;
    st.bytesLast = C.bytesRx.load();
    size_t have = 0;
    uint32_t lastTick = nowMs(), lastBeat = nowMs();
    constexpr size_t kCtlCap = lms::kMaxServerFrame + 8;

    while (C.running.load() && !C.otaPause.load() && cl.connected() && !st.reconnect) {
        const int avail = cl.available();
        if (avail > 0) {
            size_t room = kCtlCap - have;
            size_t want = (size_t)avail < room ? (size_t)avail : room;
            const int r = cl.read(C.ctlBuf + have, want);
            if (r > 0) have += (size_t)r;
        }
        for (;;) {
            lms::Frame f;
            size_t used = 0;
            const lms::FrameResult fr = lms::parseFrame(C.ctlBuf, have, f, used);
            if (fr == lms::FrameResult::NeedMore) break;
            if (fr == lms::FrameResult::Invalid) {
                Console.printf("%s protocol desync, reconnecting\n", kTag);
                st.reconnect = true;
                have = 0;
                break;
            }
            handleFrame(f, st, cl);
            memmove(C.ctlBuf, C.ctlBuf + used, have - used);
            have -= used;
        }

        const uint32_t now = nowMs();
        if (now - lastTick >= kTickMs) {
            lastTick = now;
            const uint32_t cur = C.bytesRx.load();
            st.bytesTotal += (uint32_t)(cur - st.bytesLast);
            st.bytesLast = cur;

            lms::Levels lv;
            lv.streamBytes = (uint32_t)C.ring.available();
            lv.streamEof = C.streamEof.load();
            lv.outFrames = (uint32_t)C.out.available();
            lv.decoderDone = C.decoderDone.load();
            lv.underrun = C.underrun.exchange(false);
            lv.nowMs = now;
            uint32_t ev = st.transport.tick(lv);
            if (C.evConnected.exchange(false)) ev |= st.transport.onStreamConnected();
            if (C.evHeaders.exchange(false))   ev |= st.transport.onHeaders();
            if (C.boundaryHit.exchange(false)) ev |= st.transport.onTrackBoundary();
            if (C.streamError.exchange(false) || C.decoderError.exchange(false)) {
                ev |= st.transport.onStreamError();
                abortSession();
                flushOutput();
            }
            syncOutputFlags(st);
            if (ev) {
                if (ev & lms::EV_o) C.diagRebuffers.fetch_add(1, std::memory_order_relaxed);
                sendEvents(cl, st, ev);
                lastBeat = now;
            } else if (now - lastBeat >= kHeartbeatMs) {
                sendStat(cl, st, lms::EV_t);
                lastBeat = now;
            }
            if (C.params && C.params->getMaxDacRate() != heloMaxRate &&
                (st.transport.phase() == lms::Phase::Off || st.transport.phase() == lms::Phase::Stopped)) {
                Console.printf("%s Max DAC rate changed (%u -> %u), reconnecting HELO\n",
                               kTag, (unsigned)heloMaxRate, (unsigned)C.params->getMaxDacRate());
                st.reconnect = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (cl.connected()) {
        static const uint8_t bye[9] = {'B', 'Y', 'E', '!', 0, 0, 0, 1, 0};
        cl.write(bye, sizeof(bye));
    }
    cl.stop();
    C.connected.store(false);
    stopPlayback(st);
    Console.printf("%s disconnected\n", kTag);
    if (st.redirectIp) {
        redirectOut = IPAddress((uint32_t)__builtin_bswap32(st.redirectIp));
        C.controlServer = redirectOut;
        C.serverIpRaw.store((uint32_t)redirectOut);
        hasRedirectOut = true;
    }
}

void controlTaskFn(void*) {
    uint32_t backoffMs = 1000;
    IPAddress redirect;
    bool haveRedirect = false;
    while (C.running.load(std::memory_order_acquire)) {
        if (C.otaPause.load() || WiFi.status() != WL_CONNECTED) {
            C.controlIdle.store(true);
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }
        C.controlIdle.store(false);

        IPAddress server;
        bool found = false;
        if (haveRedirect) {
            server = redirect;
            haveRedirect = false;
            found = true;
        } else {
            found = discoverServer(server);
        }
        if (found) {
            runConnection(server, redirect, haveRedirect);
            backoffMs = 1000;
        } else {
            Console.printf("%s no server found, retrying\n", kTag);
        }
        C.controlIdle.store(true);
        for (uint32_t waited = 0; waited < backoffMs && C.running.load() && !C.otaPause.load(); waited += 100) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!found && backoffMs < 15000) backoffMs *= 2;
    }
    C.controlIdle.store(true);
    C.controlExited.store(true);
    vTaskDelete(nullptr);
}

}  // namespace

// ================================================================== public API

size_t lms_output_frames(float* left, float* right, size_t frames) {
    if (!C.outMutex || !C.playing.load(std::memory_order_acquire) || C.outPaused.load(std::memory_order_acquire)) {
        C.lastAudioUs = 0;
        return 0;
    }
    C.diagAudioCalls.fetch_add(1, std::memory_order_relaxed);
    const uint32_t nowUs = (uint32_t)micros();
    if (C.lastAudioUs != 0) {
        const uint32_t dt = nowUs - C.lastAudioUs;
        if (dt > C.diagMaxAudioUs.load(std::memory_order_relaxed)) {
            C.diagMaxAudioUs.store(dt, std::memory_order_relaxed);
        }
    }
    C.lastAudioUs = nowUs;
    if (xSemaphoreTake(C.outMutex, 0) != pdTRUE) {
        C.diagMutexMiss.fetch_add(1, std::memory_order_relaxed);
        return 0;     // a flush is in progress: output silence
    }

    size_t n = C.out.read(left, right, frames);

    // skip-ahead (strm 'a'): drop frames instead of playing them
    uint32_t skip = C.skipFrames.load();
    if (skip > 0 && n > 0) {
        const uint32_t drop = skip < (uint32_t)n ? skip : (uint32_t)n;
        C.skipFrames.fetch_sub(drop);
        memmove(left, left + drop, (n - drop) * sizeof(float));
        memmove(right, right + drop, (n - drop) * sizeof(float));
        const uint32_t consumed = (uint32_t)n;
        C.framesRead.fetch_add(consumed);
        n -= drop;
        for (size_t i = n; i < frames; ++i) { left[i] = 0.0f; right[i] = 0.0f; }
    } else {
        C.framesRead.fetch_add((uint32_t)n);
    }

    if (n > 0) {
        const uint32_t played = C.trackFrames.load() + (uint32_t)n;
        C.trackFrames.store(played);
        if (C.boundaryPending.load()) {
            const uint32_t fr = C.framesRead.load();
            const int32_t over = (int32_t)(fr - C.boundaryAt.load());
            if (over >= 0) {
                C.trackFrames.store((uint32_t)over);
                C.boundaryPending.store(false);
                C.boundaryHit.store(true);
            }
        }
    }
    if (n < frames && !C.decoderDone.load()) {
        C.underrun.store(true);
        C.diagAudioShort.fetch_add(1, std::memory_order_relaxed);
        C.diagAudioShortFrames.fetch_add((uint32_t)(frames - n), std::memory_order_relaxed);
    }
    if (!C.decoderDone.load()) {
        const uint32_t av = (uint32_t)C.out.available();
        uint32_t prev = C.diagMinOutFrames.load(std::memory_order_relaxed);
        while (av < prev && !C.diagMinOutFrames.compare_exchange_weak(prev, av, std::memory_order_relaxed)) {}
    }

    const float tl = C.volL.load(std::memory_order_relaxed);
    const float tr = C.volR.load(std::memory_order_relaxed);
    if (n > 0) {
        if (C.curL == tl && C.curR == tr) {
            if (tl != 1.0f || tr != 1.0f) {
                for (size_t i = 0; i < n; ++i) { left[i] *= tl; right[i] *= tr; }
            }
        } else {
            const float stepL = (tl - C.curL) / (float)n;
            const float stepR = (tr - C.curR) / (float)n;
            float gl = C.curL, gr = C.curR;
            for (size_t i = 0; i < n; ++i) {
                gl += stepL;
                gr += stepR;
                left[i] *= gl;
                right[i] *= gr;
            }
            C.curL = tl;
            C.curR = tr;
        }
    }
    xSemaphoreGive(C.outMutex);
    return n;
}

bool lms_get_status(LmsStatus* out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    out->connected = C.connected.load();
    const uint32_t raw = C.serverIpRaw.load();
    if (out->connected && raw) {
        const IPAddress ip(raw);
        snprintf(out->serverIp, sizeof(out->serverIp), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    }
    out->phase = C.phase.load();
    out->trackFrames = C.trackFrames.load();
    out->outFrames = (uint32_t)C.out.available();
    out->streamBytes = (uint32_t)C.ring.available();
    out->sourceRate = C.srcRate.load();
    out->sourceBits = C.srcBits.load();
    return true;
}

bool lms_get_resampler_stats(resampler_stats_t* out) {
    if (!out || !C.rsMutex) return false;
    bool ok = false;
    if (xSemaphoreTake(C.rsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (C.rs) {
            resampler_get_stats(C.rs, out);
            ok = true;
        }
        xSemaphoreGive(C.rsMutex);
    }
    return ok;
}

TaskHandle_t lms_decode_task_handle(void) { return C.decodeTask; }

void lms_pause_for_ota(void) {
    C.otaPause.store(true);
    for (int i = 0; i < 150 && !C.controlIdle.load(); ++i) vTaskDelay(pdMS_TO_TICKS(10));
}

// ================================================================== LmsClient

LmsClient::LmsClient(ParamStore& params) : _params(params) {}

LmsClient::~LmsClient() { stop(); }

bool LmsClient::begin() {
    if (_running.load(std::memory_order_acquire)) return true;
    C.params = &_params;
    C.activeOutRate.store(_params.getActiveRate(), std::memory_order_release);
    Console.printf("%s starting LMS client (free int DRAM=%u, PSRAM=%u)\n",
                   kTag, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());

    C.streamStore = (uint8_t*)allocPsramFirst(kStreamRingBytes);
    C.tmpBuf      = (uint8_t*)allocPsramFirst(4096);
    C.inL         = (float*)allocPsramFirst(kChunkFrames * sizeof(float));
    C.inR         = (float*)allocPsramFirst(kChunkFrames * sizeof(float));
    C.rsL         = (float*)allocPsramFirst(kResampleOutCap * sizeof(float));
    C.rsR         = (float*)allocPsramFirst(kResampleOutCap * sizeof(float));
    C.s32         = (int32_t*)allocPsramFirst(kChunkFrames * 2 * sizeof(int32_t));
    C.pcmBuf      = (uint8_t*)allocPsramFirst(8192 + 16);
    C.ctlBuf      = (uint8_t*)allocPsramFirst(lms::kMaxServerFrame + 8);
    if (!C.streamStore || !C.tmpBuf || !C.inL || !C.inR || !C.rsL || !C.rsR || !C.s32 || !C.pcmBuf || !C.ctlBuf ||
        !C.ring.init(C.streamStore, kStreamRingBytes) || !C.out.allocate(false)) {
        Console.printf("%s buffer allocation failed\n", kTag);
        return false;
    }
    if (!C.outMutex) C.outMutex = xSemaphoreCreateMutex();
    if (!C.rsMutex)  C.rsMutex  = xSemaphoreCreateMutex();
    if (!C.outMutex || !C.rsMutex) return false;

    C.gen.store(0); C.readyGen.store(0); C.streamAck.store(0); C.decodeAck.store(0);
    resetSessionFlags();
    C.running.store(true, std::memory_order_release);
    C.controlExited.store(false); C.streamExited.store(false); C.decodeExited.store(false);

    const bool tasksOk =
        (xTaskCreatePinnedToCore(streamTaskFn, "lms_stream", 6144, nullptr, 2, &C.streamTask, 0) == pdPASS) &&
        (xTaskCreatePinnedToCore(decodeTaskFn, "lms_decode", 8192, nullptr, 3, &C.decodeTask, 0) == pdPASS) &&
        (xTaskCreatePinnedToCore(controlTaskFn, "lms_ctl", 5120, nullptr, 3, &C.controlTask, 0) == pdPASS);
    if (!tasksOk) {
        Console.printf("%s task creation failed (free int DRAM=%u)\n", kTag, (unsigned)ESP.getFreeHeap());
        C.running.store(false);
        return false;
    }

    _running.store(true, std::memory_order_release);
    _telemetryExited.store(false, std::memory_order_release);
    if (xTaskCreatePinnedToCore(_telemetryTaskTrampoline, "lms_telemetry", 3072, this, 1, &_telemetryTaskHandle, 0) != pdPASS) {
        _telemetryTaskHandle = nullptr;
        _telemetryExited.store(true, std::memory_order_release);
    }
    Console.printf("%s tasks started (free int DRAM=%u, min=%u)\n",
                   kTag, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
    return true;
}

void LmsClient::stop() {
    if (!_running.exchange(false, std::memory_order_acq_rel)) return;
    C.running.store(false, std::memory_order_release);
    for (int w = 0; w < 400 && !(C.controlExited.load() && C.streamExited.load() && C.decodeExited.load() && _telemetryExited.load()); ++w) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    _telemetryTaskHandle = nullptr;
    C.playing.store(false);
    Console.printf("%s client stopped\n", kTag);
}

void LmsClient::_telemetryTaskTrampoline(void* arg) {
    static_cast<LmsClient*>(arg)->_runTelemetry();
}

void LmsClient::_runTelemetry() {
    uint32_t lastPrintMs = 0;
    uint32_t idleSinceMs = 0;
    bool wasStreaming = false;

    while (_running.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(250));

        WiFiStreamMetadata meta = _params.getWiFiStreamMetadata();
        LmsStatus st;
        lms_get_status(&st);
        const lms::Phase phase = (lms::Phase)st.phase;

        const bool isStream = (phase == lms::Phase::Playing || phase == lms::Phase::Buffering);
        const bool isPaused = (phase == lms::Phase::Paused);

        meta.connected = st.connected;
        snprintf(meta.serverIp, sizeof(meta.serverIp), "%s", st.serverIp);
        if (meta.connected && meta.serverIp[0]) {
            snprintf(meta.serverName, sizeof(meta.serverName), "LMS (%s)", meta.serverIp);
        } else {
            snprintf(meta.serverName, sizeof(meta.serverName), "None");
        }
        meta.streaming  = isStream;
        meta.paused     = isPaused;
        const uint32_t outRate = C.activeOutRate.load(std::memory_order_acquire);
        meta.sampleRate = st.sourceRate ? st.sourceRate : outRate;
        meta.bitDepth   = st.sourceBits ? st.sourceBits : 16;
        meta.pos        = outRate ? (st.trackFrames / outRate) : 0;

        if (isStream) {
            idleSinceMs = 0;
            const unsigned srcRate = meta.sampleRate;
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "%u.%u kHz %u-bit",
                     srcRate / 1000, (srcRate % 1000) / 100, (unsigned)meta.bitDepth);
            if (!wasStreaming && _params.getInputMode() != INPUT_MODE_WIFI) {
                Console.println("[LMS] Auto-switching input mode to Squeezebox");
                _params.setInputMode(INPUT_MODE_WIFI, false);
            }
        } else {
            snprintf(meta.formatInfo, sizeof(meta.formatInfo), "Idle");
            if (!isPaused && _params.getInputMode() == INPUT_MODE_WIFI &&
                (phase == lms::Phase::Off || phase == lms::Phase::Stopped)) {
                if (idleSinceMs == 0) idleSinceMs = millis();
                if (millis() - idleSinceMs >= 3000) {
                    _params.setInputMode(INPUT_MODE_AUX, false);
                    idleSinceMs = 0;
                }
            } else {
                idleSinceMs = 0;
            }
        }

        wasStreaming = isStream;
        _params.setWiFiStreamMetadata(meta);

        const uint32_t now = millis();
        const bool hasAnomaly =
            (C.diagAudioShort.load(std::memory_order_relaxed) > 0) ||
            (C.diagRebuffers.load(std::memory_order_relaxed) > 0) ||
            (C.diagMutexMiss.load(std::memory_order_relaxed) > 0);
        if (isStream && (hasAnomaly || (now - lastPrintMs >= 5000))) {
            lastPrintMs = now;
            wifi_ps_type_t ps = WIFI_PS_NONE;
            esp_wifi_get_ps(&ps);
            const uint32_t shrt    = C.diagAudioShort.exchange(0, std::memory_order_relaxed);
            const uint32_t shrtFr  = C.diagAudioShortFrames.exchange(0, std::memory_order_relaxed);
            const uint32_t mtxMiss = C.diagMutexMiss.exchange(0, std::memory_order_relaxed);
            const uint32_t maxAud  = C.diagMaxAudioUs.exchange(0, std::memory_order_relaxed);
            const uint32_t maxDec  = C.diagMaxDecodeMs.exchange(0, std::memory_order_relaxed);
            const uint32_t maxRw   = C.diagMaxRingWaitMs.exchange(0, std::memory_order_relaxed);
            const uint32_t maxOw   = C.diagMaxOutWaitMs.exchange(0, std::memory_order_relaxed);
            const uint32_t minRb   = C.diagMinRingBytes.exchange(0xFFFFFFFFu, std::memory_order_relaxed);
            const uint32_t minOf   = C.diagMinOutFrames.exchange(0xFFFFFFFFu, std::memory_order_relaxed);
            const uint32_t rebuf   = C.diagRebuffers.exchange(0, std::memory_order_relaxed);
            C.diagAudioCalls.store(0, std::memory_order_relaxed);
            Console.printf("%s phase=%d | sbuf=%uKB | obuf=%u | pos=%us | short=%u(%ufr) mtx=%u rebuf=%u | maxAud=%uus maxDec=%ums maxRw=%ums maxOw=%ums | minRb=%u minOf=%u | rssi=%d dBm | ps=%d\n",
                           hasAnomaly ? "[LMS WARN]" : "[LMS STATS]",
                           (int)phase, (unsigned)(st.streamBytes / 1024), (unsigned)st.outFrames, (unsigned)meta.pos,
                           (unsigned)shrt, (unsigned)shrtFr, (unsigned)mtxMiss, (unsigned)rebuf,
                           (unsigned)maxAud, (unsigned)maxDec, (unsigned)maxRw, (unsigned)maxOw,
                           minRb == 0xFFFFFFFFu ? 0u : (unsigned)minRb,
                           minOf == 0xFFFFFFFFu ? 0u : (unsigned)minOf,
                           (int)WiFi.RSSI(), (int)ps);
        }
    }

    _telemetryExited.store(true, std::memory_order_release);
    vTaskDelete(nullptr);
}
