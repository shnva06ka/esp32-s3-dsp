#include <Arduino.h>
#include <unity.h>
#include <cmath>
#include <cstring>
#include "audio_config.h"
#include "audio_ring_buffer.h"
#include "dsp_pipeline.h"
#include "param_store.h"
#include "lms_codecs.h"
#include "system_services.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_netif.h>

extern esp_netif_t* get_esp_interface_netif(esp_interface_t interface);

extern "C" {
int config_get_wifi_cache_tx_buffer_num(void);
int config_get_wifi_dynamic_tx_buffer_num(void);
int32_t test_drflac_pred32(uint32_t order, int32_t shift, const int32_t* coeffs, int32_t* decodedSamples);
int32_t test_drflac_pred64(uint32_t order, int32_t shift, const int32_t* coeffs, int32_t* decodedSamples);
uint32_t test_drflac_clz(uint32_t x);
uint32_t test_drflac_ilog2(uint32_t x);
void console_log(const char* buf) {
    if (buf) {
        Serial.print(buf);
    }
}
#include "../../src/lms_codecs.c"
#include "../../src/lwip_tcp_tuning.c"
}

void test_ring_buffer_basic_ops(void) {
    AudioRingBuffer<128> rb;
    TEST_ASSERT_TRUE(rb.allocate());
    TEST_ASSERT_EQUAL_UINT32(128, rb.capacity());
    TEST_ASSERT_EQUAL_UINT32(0, rb.available());

    float inL[64], inR[64];
    for (int i = 0; i < 64; ++i) {
        inL[i] = static_cast<float>(i);
        inR[i] = static_cast<float>(-i);
    }

    size_t written = rb.write(inL, inR, 64);
    TEST_ASSERT_EQUAL_UINT32(64, written);
    TEST_ASSERT_EQUAL_UINT32(64, rb.available());

    float outL[64]{0}, outR[64]{0};
    size_t read = rb.read(outL, outR, 32);
    TEST_ASSERT_EQUAL_UINT32(32, read);
    TEST_ASSERT_EQUAL_UINT32(32, rb.available());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, outL[0]);
    TEST_ASSERT_EQUAL_FLOAT(31.0f, outL[31]);

    size_t skipped = rb.skip(16);
    TEST_ASSERT_EQUAL_UINT32(16, skipped);
    TEST_ASSERT_EQUAL_UINT32(16, rb.available());

    read = rb.read(outL, outR, 16);
    TEST_ASSERT_EQUAL_UINT32(16, read);
    TEST_ASSERT_EQUAL_UINT32(0, rb.available());
    TEST_ASSERT_EQUAL_FLOAT(48.0f, outL[0]);
}

void test_ring_buffer_wrap_and_bounds(void) {
    AudioRingBuffer<64> rb;
    TEST_ASSERT_TRUE(rb.allocate());

    TEST_ASSERT_EQUAL_UINT32(0, rb.write(nullptr, nullptr, 16));

    float inL[64], inR[64];
    for (int i = 0; i < 64; ++i) { inL[i] = static_cast<float>(i + 1); inR[i] = -static_cast<float>(i + 1); }

    TEST_ASSERT_EQUAL_UINT32(63, rb.write(inL, inR, 64));
    TEST_ASSERT_EQUAL_UINT32(0, rb.space());

    float outL[64], outR[64];
    TEST_ASSERT_EQUAL_UINT32(32, rb.read(outL, outR, 32));
    TEST_ASSERT_EQUAL_UINT32(32, rb.write(inL, inR, 40));

    rb.clear();
    TEST_ASSERT_EQUAL_UINT32(0, rb.available());
    for (int i = 0; i < 16; ++i) { outL[i] = 99.0f; outR[i] = -99.0f; }
    TEST_ASSERT_EQUAL_UINT32(0, rb.read(outL, outR, 16));
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, outL[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, outR[i]);
    }
}

void test_iir_eq_nan_denormal(void) {
    IirEQ eq;
    eq.setBand(0, { 1000.0f, 6.0f, 1.0f, true, EQ_FILTER_PEAK, 2 });
    eq.setBand(1, { NAN, NAN, NAN, true, EQ_FILTER_PEAK, 2 });

    float left[32], right[32];
    for (int i = 0; i < 32; ++i) {
        left[i] = 0.5f;
        right[i] = -0.5f;
    }
    left[4] = NAN;
    right[8] = INFINITY;

    eq.process(left, right, 32, 2);

    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
    }

    for (int i = 0; i < 32; ++i) {
        left[i] = 1e-20f;
        right[i] = -1e-25f;
    }
    eq.process(left, right, 32, 2);
    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
    }
}

void test_iir_eq_20hz_and_subsonic_96khz(void) {
    SubsonicFilter sub;
    sub.setParams({ 20.0f, 4, true });

    const size_t N = 2048;
    float l[256], r[256];
    double rms1k = 0.0;
    for (size_t blk = 0; blk < N / 256; ++blk) {
        for (size_t i = 0; i < 256; ++i) {
            size_t n = blk * 256 + i;
            l[i] = 0.5f * sinf(2.0f * (float)M_PI * 1000.0f * (float)n / (float)kSampleRate);
            r[i] = l[i];
        }
        sub.process(l, r, 256);
        for (size_t i = 0; i < 256; ++i) {
            TEST_ASSERT_TRUE(std::isfinite(l[i]));
            if (blk >= 4) rms1k += l[i] * l[i];
        }
    }
    rms1k = sqrt(rms1k / (4.0 * 256.0));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.35355f, (float)rms1k);

    sub.reset();
    for (int blk = 0; blk < 120; ++blk) {
        for (int i = 0; i < 256; ++i) { l[i] = 1.0f; r[i] = 1.0f; }
        sub.process(l, r, 256);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l[255]);
}

void test_speaker_correction_hilbert_and_delay(void) {
    static SpeakerCorrection sc;
    const float testFreqs[3] = { 100.0f, 1000.0f, 10000.0f };
    for (int fIdx = 0; fIdx < 3; ++fIdx) {
        sc.reset();
        SubsonicParams sp{ 20.0f, 2, true };
        sp.balance = 0.0f;
        sp.delayMsL = -1.0f;
        sp.delayMsR =  0.0f;
        sp.phaseL = 45;
        sp.phaseR = -90;
        sc.setParams(sp);

        float l[256], r[256];
        double sumL2 = 0.0, sumR2 = 0.0;
        const float freq = testFreqs[fIdx];
        for (int blk = 0; blk < 40; ++blk) {
            for (int i = 0; i < 256; ++i) {
                float s = 0.5f * sinf(2.0f * (float)M_PI * freq * (float)(blk * 256 + i) / (float)kSampleRate);
                l[i] = s;
                r[i] = s;
            }
            sc.process(l, r, 256);
            if (blk >= 30) {
                for (int i = 0; i < 256; ++i) {
                    TEST_ASSERT_TRUE(std::isfinite(l[i]));
                    TEST_ASSERT_TRUE(std::isfinite(r[i]));
                    sumL2 += l[i] * l[i];
                    sumR2 += r[i] * r[i];
                }
            }
        }
        float rmsL = sqrtf((float)(sumL2 / (10.0 * 256.0)));
        float rmsR = sqrtf((float)(sumR2 / (10.0 * 256.0)));
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.35355f, rmsL);
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.35355f, rmsR);
    }
}

void test_compander_and_limiter(void) {
    Compander comp;
    comp.setParams({ -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true });

    float left[32], right[32];
    for (int i = 0; i < 32; ++i) {
        left[i] = 2.0f;
        right[i] = 2.5f;
    }
    comp.process(left, right, 32);

    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
    }

    SoftLimiter lim;
    lim.setParams({ -0.5f, 3.0f, true });
    lim.process(left, right, 32);

    const float expectedCeiling = powf(10.0f, -0.5f / 20.0f);
    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
        TEST_ASSERT_TRUE(fabsf(left[i]) <= expectedCeiling + 1e-4f);
        TEST_ASSERT_TRUE(fabsf(right[i]) <= expectedCeiling + 1e-4f);
    }

    const float kneeLevelsDb[3] = { -1.5f, -0.5f, 0.5f };
    for (int idx = 0; idx < 3; ++idx) {
        const float levelDb = kneeLevelsDb[idx];
        const float peak = powf(10.0f, levelDb / 20.0f);
        float kL[4] = { peak, -peak, peak * 0.25f, -peak * 0.5f };
        float kR[4] = { peak * 0.5f, -peak * 0.25f, peak, -peak };
        lim.process(kL, kR, 4);

        const float t = (levelDb - (-2.0f)) / 3.0f;
        const float expectedGainDb = -0.5f * 3.0f * t * t;
        const float expectedGain = powf(10.0f, expectedGainDb / 20.0f);

        const float g0 = fabsf(kL[0]) / peak;
        const float g1 = fabsf(kL[1]) / peak;
        const float g2 = fabsf(kR[2]) / peak;
        const float g3 = fabsf(kR[3]) / peak;
        TEST_ASSERT_FLOAT_WITHIN(0.005f, expectedGain, g0);
        TEST_ASSERT_FLOAT_WITHIN(0.005f, expectedGain, g1);
        TEST_ASSERT_FLOAT_WITHIN(0.005f, expectedGain, g2);
        TEST_ASSERT_FLOAT_WITHIN(0.005f, expectedGain, g3);
        TEST_ASSERT_FLOAT_WITHIN(0.05f, expectedGainDb, 20.0f * log10f(g0));
    }

    float mixL[64], mixR[64];
    for (int i = 0; i < 64; ++i) {
        mixL[i] = (i % 2 == 0) ? 0.5f : -1.5f;
        mixR[i] = (i % 3 == 0) ? 0.9f : -0.3f;
    }
    mixL[0]  = INFINITY;  mixR[0]  = 0.4f;
    mixL[7]  = -INFINITY; mixR[7]  = -INFINITY;
    mixL[15] = NAN;       mixR[15] = INFINITY;
    mixL[31] = 0.95f;     mixR[31] = NAN;
    mixL[47] = -INFINITY; mixR[47] = 1.2f;
    mixL[63] = NAN;       mixR[63] = -NAN;
    lim.process(mixL, mixR, 64);
    for (int i = 0; i < 64; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(mixL[i]));
        TEST_ASSERT_TRUE(std::isfinite(mixR[i]));
        TEST_ASSERT_TRUE(fabsf(mixL[i]) <= expectedCeiling + 1e-4f);
        TEST_ASSERT_TRUE(fabsf(mixR[i]) <= expectedCeiling + 1e-4f);
    }

    comp.reset();
    comp.setParams({ -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true });
    float ramp256L[256], ramp256R[256];
    for (int i = 0; i < 256; ++i) {
        ramp256L[i] = 0.5f;
        ramp256R[i] = -0.5f;
    }
    comp.process(ramp256L, ramp256R, 256);
    double ramp256Sum = 0.0;
    for (int i = 0; i < 256; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(ramp256L[i]));
        TEST_ASSERT_TRUE(std::isfinite(ramp256R[i]));
        ramp256Sum += static_cast<double>(ramp256L[i]) * static_cast<double>(ramp256L[i]);
    }
    const float ramp256Rms = sqrtf(static_cast<float>(ramp256Sum / 256.0));
    TEST_ASSERT_TRUE(ramp256Rms < 0.5f && ramp256Rms > 0.1f);
    TEST_ASSERT_TRUE(fabsf(ramp256L[255]) < fabsf(ramp256L[0]));

    comp.reset();
    float ramp35L[35], ramp35R[35];
    for (int i = 0; i < 35; ++i) {
        ramp35L[i] = 2.0f;
        ramp35R[i] = -2.0f;
    }
    comp.process(ramp35L, ramp35R, 35);
    double ramp35Sum = 0.0;
    for (int i = 0; i < 35; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(ramp35L[i]));
        TEST_ASSERT_TRUE(std::isfinite(ramp35R[i]));
        ramp35Sum += static_cast<double>(ramp35L[i]) * static_cast<double>(ramp35L[i]);
    }
    const float ramp35Rms = sqrtf(static_cast<float>(ramp35Sum / 35.0));
    TEST_ASSERT_TRUE(ramp35Rms < 2.0f && ramp35Rms > 0.5f);
    TEST_ASSERT_TRUE(fabsf(ramp35L[34]) < fabsf(ramp35L[31]));
    TEST_ASSERT_TRUE(fabsf(ramp35L[34]) < fabsf(ramp35L[0]));

    lim.setParams({ NAN, NAN, true });
    for (int i = 0; i < 32; ++i) { left[i] = 3.0f; right[i] = -3.0f; }
    lim.process(left, right, 32);
    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(fabsf(left[i]) <= expectedCeiling + 1e-4f);
    }
}

void test_dsp_pipeline_and_audio_quality(void) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Eq, (uint8_t)parseDspModuleId("eq"));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Subsonic, (uint8_t)parseDspModuleId("sub"));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Unknown, (uint8_t)parseDspModuleId("invalid"));

    static ParamStore ps;
    ps.setModuleOrder("eq,eq,sub,comp,lim");
    TEST_ASSERT_EQUAL_STRING("eq,subsonic,comp,lim", ps.getModuleOrder().c_str());

    static DSPPipeline pipe(ps);
    float l[256], r[256], ref[256];
    double errSq = 0.0, refSq = 0.0;
    for (int i = 0; i < 256; ++i) {
        ref[i] = 0.5f * sinf(2.0f * (float)M_PI * 1000.0f * (float)i / (float)kSampleRate);
        l[i] = ref[i];
        r[i] = ref[i];
    }
    pipe.process(l, r, 256);
    for (int i = 0; i < 256; ++i) {
        double d = (double)l[i] - (double)ref[i];
        errSq += d * d;
        refSq += (double)ref[i] * (double)ref[i];
    }
    TEST_ASSERT_TRUE(errSq / refSq < 1e-12);
}

void test_param_store_sanitization(void) {
    static ParamStore ps;

    EQBandParams pBad{ -500.0f, 100.0f, -5.0f, true, 99, 10 };
    ps.setEQBand(0, pBad);

    EQBandParams pSan = ps.getEQBand(0);
    TEST_ASSERT_TRUE(pSan.freq >= 10.0f);
    TEST_ASSERT_TRUE(pSan.gain <= 24.0f);
    TEST_ASSERT_TRUE(pSan.q >= 0.05f);
    TEST_ASSERT_TRUE(pSan.type <= EQ_FILTER_FLAT_HIGH_PASS);
    TEST_ASSERT_EQUAL_UINT8(2, pSan.order);

    TEST_ASSERT_TRUE(ps.addEQBand(NAN, 999.0f, -1.0f, 88, 9));
    EQBandParams added = ps.getEQBand(ps.getEQBandCount() - 1);
    TEST_ASSERT_TRUE(std::isfinite(added.freq) && added.freq >= 10.0f);
    TEST_ASSERT_TRUE(std::isfinite(added.gain) && added.gain <= 24.0f);
    TEST_ASSERT_TRUE(std::isfinite(added.q) && added.q >= 0.05f);
    TEST_ASSERT_EQUAL_UINT8(EQ_FILTER_PEAK, added.type);
    TEST_ASSERT_EQUAL_UINT8(2, added.order);

    SubsonicParams sub{ 30.0f, 4, true };
    sub.delayMsL = -10.0f;
    sub.phaseL = -90;
    ps.setSubsonic(sub);
    SubsonicParams subOut = ps.getSubsonic();
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, subOut.delayMsL);
    TEST_ASSERT_EQUAL_INT16(-90, subOut.phaseL);

    ps.setPreamp(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(24.0f, ps.getPreamp());
    ps.setPreamp(-100.0f);
    TEST_ASSERT_EQUAL_FLOAT(-24.0f, ps.getPreamp());
    ps.setPreamp(NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ps.getPreamp());
}

void test_resampler_arbitrary_rate(void) {
    resampler_t* r = resampler_create(44100, 96000);
    TEST_ASSERT_NOT_NULL(r);

    TEST_ASSERT_EQUAL_UINT32(0, (uint32_t)resampler_in_for_out(r, 0));

    size_t inFrames = 256;
    size_t maxOut = resampler_max_out(r, inFrames);
    TEST_ASSERT_TRUE(maxOut > 500 && maxOut < 700);

    static float inL[256], inR[256];
    static float outL[768], outR[768];
    double rmsOut = 0.0;
    size_t totalOut = 0;

    for (int blk = 0; blk < 8; ++blk) {
        for (size_t i = 0; i < inFrames; ++i) {
            size_t idx = (size_t)blk * inFrames + i;
            inL[i] = 0.8f * sinf(2.0f * (float)M_PI * 1000.0f * (float)idx / 44100.0f);
            inR[i] = 0.8f * cosf(2.0f * (float)M_PI * 1000.0f * (float)idx / 44100.0f);
        }
        size_t produced = resampler_process(r, inL, inR, inFrames, outL, outR, 768);
        TEST_ASSERT_TRUE(produced > 0);
        for (size_t i = 0; i < produced; ++i) {
            TEST_ASSERT_TRUE(std::isfinite(outL[i]));
            TEST_ASSERT_TRUE(std::isfinite(outR[i]));
            if (blk >= 2) {
                rmsOut += outL[i] * outL[i];
                totalOut++;
            }
        }
    }
    rmsOut = sqrt(rmsOut / (double)totalOut);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5657f, (float)rmsOut);

    resampler_stats_t stats;
    resampler_get_stats(r, &stats);
    TEST_ASSERT_EQUAL_UINT32(44100, stats.in_rate);
    TEST_ASSERT_EQUAL_UINT32(96000, stats.out_rate);
    TEST_ASSERT_EQUAL_UINT32(8U, static_cast<uint32_t>(stats.total_calls));

    resampler_destroy(r);

    resampler_run_self_test();
}

void test_wav_aiff_parser_boundaries(void) {
    const uint8_t ieee44k[10] = { 0x40, 0x0E, 0xAC, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int expon = (((int)ieee44k[0] & 0x7F) << 8) | ieee44k[1];
    uint32_t hi = ((uint32_t)ieee44k[2] << 24) | ((uint32_t)ieee44k[3] << 16) | ((uint32_t)ieee44k[4] << 8) | ieee44k[5];
    expon -= 16383;
    unsigned rate = (unsigned)(ldexp((double)hi, expon - 31) + 0.5);
    TEST_ASSERT_EQUAL_UINT32(44100, rate);

    uint32_t chunkLen = 100;
    uint32_t maliciousOffset = 0xFFFFFFFCu;
    bool validSsnd = (chunkLen >= 8 && (uint64_t)maliciousOffset <= (uint64_t)chunkLen - 8ULL);
    TEST_ASSERT_FALSE(validSsnd);
}

void test_ap_fallback_isolation_and_captive_portal(void) {
    TEST_ASSERT_TRUE(apIsCaptivePortalProbePath("/generate_204"));
    TEST_ASSERT_TRUE(apIsCaptivePortalProbePath("/gen_204"));
    TEST_ASSERT_TRUE(apIsCaptivePortalProbePath("/hotspot-detect.html"));
    TEST_ASSERT_TRUE(apIsCaptivePortalProbePath("/connecttest.txt"));
    TEST_ASSERT_TRUE(apIsCaptivePortalProbePath("/ncsi.txt"));
    TEST_ASSERT_FALSE(apIsCaptivePortalProbePath("/"));
    TEST_ASSERT_FALSE(apIsCaptivePortalProbePath("/api/wifi"));
    TEST_ASSERT_TRUE(apIsLocalDspHostname("dsp"));
    TEST_ASSERT_TRUE(apIsLocalDspHostname("DSP.LOCAL"));
    TEST_ASSERT_TRUE(apIsLocalDspHostname("esp32-s3-dsp.local"));
    TEST_ASSERT_FALSE(apIsLocalDspHostname("connectivitycheck.gstatic.com"));
    TEST_ASSERT_FALSE(apIsLocalDspHostname("example.com"));
    TEST_ASSERT_FALSE(apIsLocalDspHostname(""));

    TEST_ASSERT_TRUE(apShouldSendWsTelemetry(true, true, 0));
    TEST_ASSERT_FALSE(apShouldSendWsTelemetry(true, true, 8));
    TEST_ASSERT_FALSE(apShouldSendWsTelemetry(false, true, 0));

    const IPAddress apIp(192, 168, 4, 1);
    uint8_t dnsOut[512]{};

    const uint8_t queryDspA[] = {
        0x12, 0x34, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 'd', 's', 'p', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
        0x00, 0x01, 0x00, 0x01
    };
    size_t outLen = apBuildCaptiveDnsResponse(queryDspA, sizeof(queryDspA), apIp, dnsOut, sizeof(dnsOut));
    TEST_ASSERT_EQUAL_UINT32(sizeof(queryDspA) + 16U, outLen);
    TEST_ASSERT_EQUAL_UINT8(0x12, dnsOut[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, dnsOut[1]);
    TEST_ASSERT_TRUE((dnsOut[2] & 0x80) != 0);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[3] & 0x0F);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[5]);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[7]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[11]);
    TEST_ASSERT_EQUAL_UINT8(192, dnsOut[outLen - 4]);
    TEST_ASSERT_EQUAL_UINT8(168, dnsOut[outLen - 3]);
    TEST_ASSERT_EQUAL_UINT8(4,   dnsOut[outLen - 2]);
    TEST_ASSERT_EQUAL_UINT8(1,   dnsOut[outLen - 1]);

    const uint8_t queryDspEdnsA[] = {
        0x56, 0x78, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x03, 'd', 's', 'p', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
        0x00, 0x01, 0x00, 0x01,
        0x00, 0x00, 0x29, 0x04, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    outLen = apBuildCaptiveDnsResponse(queryDspEdnsA, sizeof(queryDspEdnsA), apIp, dnsOut, sizeof(dnsOut));
    TEST_ASSERT_EQUAL_UINT32(sizeof(queryDspA) + 16U, outLen);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[3] & 0x0F);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[5]);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[7]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[11]);
    TEST_ASSERT_EQUAL_UINT8(192, dnsOut[outLen - 4]);
    TEST_ASSERT_EQUAL_UINT8(168, dnsOut[outLen - 3]);
    TEST_ASSERT_EQUAL_UINT8(4,   dnsOut[outLen - 2]);
    TEST_ASSERT_EQUAL_UINT8(1,   dnsOut[outLen - 1]);

    const uint8_t queryDspAAAA[] = {
        0x9A, 0xBC, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x03, 'd', 's', 'p', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
        0x00, 0x1C, 0x00, 0x01,
        0x00, 0x00, 0x29, 0x04, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    outLen = apBuildCaptiveDnsResponse(queryDspAAAA, sizeof(queryDspAAAA), apIp, dnsOut, sizeof(dnsOut));
    TEST_ASSERT_EQUAL_UINT32(sizeof(queryDspA), outLen);
    TEST_ASSERT_TRUE((dnsOut[2] & 0x80) != 0);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[3] & 0x0F);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[5]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[7]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[11]);

    const uint8_t queryExternalA[] = {
        0xDE, 0xAD, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
        0x07, 'e', 'x', 'a', 'm', 'p', 'l', 'e', 0x03, 'c', 'o', 'm', 0x00,
        0x00, 0x01, 0x00, 0x01,
        0x00, 0x00, 0x29, 0x04, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    outLen = apBuildCaptiveDnsResponse(queryExternalA, sizeof(queryExternalA), apIp, dnsOut, sizeof(dnsOut));
    TEST_ASSERT_EQUAL_UINT32(29U, outLen);
    TEST_ASSERT_TRUE((dnsOut[2] & 0x80) != 0);
    TEST_ASSERT_EQUAL_UINT8(3, dnsOut[3] & 0x0F);
    TEST_ASSERT_EQUAL_UINT8(1, dnsOut[5]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[7]);
    TEST_ASSERT_EQUAL_UINT8(0, dnsOut[11]);

    const uint8_t queryShort[] = { 0x00, 0x01, 0x01, 0x00 };
    TEST_ASSERT_EQUAL_UINT32(0U, apBuildCaptiveDnsResponse(queryShort, sizeof(queryShort), apIp, dnsOut, sizeof(dnsOut)));
    const uint8_t queryTruncLabel[] = {
        0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x20, 'a', 'b', 'c'
    };
    TEST_ASSERT_EQUAL_UINT32(0U, apBuildCaptiveDnsResponse(queryTruncLabel, sizeof(queryTruncLabel), apIp, dnsOut, sizeof(dnsOut)));

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin("__non_existent_dsp_test_ssid__", "12345678");
    delay(500);

    apEnterPureSoftApMode("ESP32-S3-DSP");

    delay(3200);

    TEST_ASSERT_EQUAL(WIFI_AP, WiFi.getMode());
    TEST_ASSERT_FALSE(WiFi.getAutoReconnect());

    wifi_config_t conf{};
    esp_wifi_get_config(WIFI_IF_STA, &conf);
    TEST_ASSERT_EQUAL_UINT8(0, conf.sta.ssid[0]);

    TEST_ASSERT_FALSE(apIsStaAssociated());
    TEST_ASSERT_EQUAL_STRING("192.168.4.1", WiFi.softAPIP().toString().c_str());
    TEST_ASSERT_GREATER_OR_EQUAL_INT(32, config_get_wifi_cache_tx_buffer_num());
    TEST_ASSERT_GREATER_OR_EQUAL_INT(32, config_get_wifi_dynamic_tx_buffer_num());

    esp_netif_t* apNetif = get_esp_interface_netif(ESP_IF_WIFI_AP);
    TEST_ASSERT_NOT_NULL(apNetif);
    esp_netif_dhcp_status_t dhcpStatus = ESP_NETIF_DHCP_INIT;
    TEST_ASSERT_EQUAL(ESP_OK, esp_netif_dhcps_get_status(apNetif, &dhcpStatus));
    TEST_ASSERT_EQUAL(ESP_NETIF_DHCP_STARTED, dhcpStatus);

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
}

void test_drflac_lpc_and_bitops(void) {
    TEST_ASSERT_EQUAL_UINT32(32U, test_drflac_clz(0U));
    TEST_ASSERT_EQUAL_UINT32(31U, test_drflac_clz(1U));
    TEST_ASSERT_EQUAL_UINT32(0U,  test_drflac_clz(0x80000000U));
    TEST_ASSERT_EQUAL_UINT32(15U, test_drflac_clz(0x00010000U));

    TEST_ASSERT_EQUAL_UINT32(0U,  test_drflac_ilog2(0U));
    TEST_ASSERT_EQUAL_UINT32(1U,  test_drflac_ilog2(1U));
    TEST_ASSERT_EQUAL_UINT32(2U,  test_drflac_ilog2(2U));
    TEST_ASSERT_EQUAL_UINT32(2U,  test_drflac_ilog2(3U));
    TEST_ASSERT_EQUAL_UINT32(4U,  test_drflac_ilog2(8U));
    TEST_ASSERT_EQUAL_UINT32(6U,  test_drflac_ilog2(32U));
    TEST_ASSERT_EQUAL_UINT32(32U, test_drflac_ilog2(0xFFFFFFFFU));

    int32_t coeffs32[32];
    int32_t buf32[64];
    for (int i = 0; i < 32; ++i) {
        coeffs32[i] = ((i & 1) ? -1 : 1) * (120 + i * 29);
    }
    for (int i = 0; i < 64; ++i) {
        buf32[i] = ((i % 3 == 0) ? -1 : 1) * (1500 + i * 137);
    }
    int32_t* pSamples32 = &buf32[32];
    const uint32_t orders32[] = { 1U, 4U, 7U, 12U, 32U };
    const int32_t shifts32[] = { 0, 8, 10 };
    for (size_t oi = 0; oi < sizeof(orders32) / sizeof(orders32[0]); ++oi) {
        const uint32_t order = orders32[oi];
        for (size_t si = 0; si < sizeof(shifts32) / sizeof(shifts32[0]); ++si) {
            const int32_t shift = shifts32[si];
            int64_t ref = 0;
            for (uint32_t j = 0; j < order; ++j) {
                ref += static_cast<int64_t>(coeffs32[j]) * static_cast<int64_t>(pSamples32[-1 - static_cast<int>(j)]);
            }
            const int32_t expected = static_cast<int32_t>(ref >> shift);
            const int32_t actual = test_drflac_pred32(order, shift, coeffs32, pSamples32);
            TEST_ASSERT_EQUAL_INT32(expected, actual);
        }
    }

    int32_t coeffs64[32];
    int32_t buf64[64];
    for (int i = 0; i < 32; ++i) {
        coeffs64[i] = ((i & 1) ? -1 : 1) * (2048 + i * 731);
    }
    for (int i = 0; i < 64; ++i) {
        buf64[i] = ((i % 3 == 0) ? 1 : -1) * (6000000 + (i % 7) * 333333);
    }
    int32_t* pSamples64 = &buf64[32];
    const uint32_t orders64[] = { 1U, 5U, 8U, 11U, 16U, 24U };
    const int32_t shifts64[] = { 12, 14, 15 };
    for (size_t oi = 0; oi < sizeof(orders64) / sizeof(orders64[0]); ++oi) {
        const uint32_t order = orders64[oi];
        for (size_t si = 0; si < sizeof(shifts64) / sizeof(shifts64[0]); ++si) {
            const int32_t shift = shifts64[si];
            int64_t ref = 0;
            for (uint32_t j = 0; j < order; ++j) {
                ref += static_cast<int64_t>(coeffs64[j]) * static_cast<int64_t>(pSamples64[-1 - static_cast<int>(j)]);
            }
            const int32_t expected = static_cast<int32_t>(ref >> shift);
            const int32_t actual = test_drflac_pred64(order, shift, coeffs64, pSamples64);
            TEST_ASSERT_EQUAL_INT32(expected, actual);
        }
    }
}

void setup() {
    Serial.begin(115200);
    unsigned long waitStart = millis();
    while (!Serial && (millis() - waitStart) < 15000UL) {
        delay(50);
    }
    delay(400);
    UNITY_BEGIN();
    RUN_TEST(test_ring_buffer_basic_ops);
    RUN_TEST(test_ring_buffer_wrap_and_bounds);
    RUN_TEST(test_iir_eq_nan_denormal);
    RUN_TEST(test_iir_eq_20hz_and_subsonic_96khz);
    RUN_TEST(test_speaker_correction_hilbert_and_delay);
    RUN_TEST(test_compander_and_limiter);
    RUN_TEST(test_dsp_pipeline_and_audio_quality);
    RUN_TEST(test_param_store_sanitization);
    RUN_TEST(test_resampler_arbitrary_rate);
    RUN_TEST(test_wav_aiff_parser_boundaries);
    RUN_TEST(test_ap_fallback_isolation_and_captive_portal);
    RUN_TEST(test_drflac_lpc_and_bitops);
    UNITY_END();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
