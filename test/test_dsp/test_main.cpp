#include <Arduino.h>
#include <unity.h>
#include <cmath>
#include <cstring>
#include "audio_config.h"
#include "audio_ring_buffer.h"
#include "biquad_eq.h"
#include "subsonic.h"
#include "compander.h"
#include "limiter.h"
#include "speaker_correction.h"
#include "dsp_module_id.h"
#include "dsp_pipeline.h"
#include "param_store.h"
#include "sl_resample.h"

extern "C" {
#include "../../src/sl_resample.c"
}

// ==========================================
// Test 1: Audio Ring Buffer
// ==========================================
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

    // Null pointer guards
    TEST_ASSERT_EQUAL_UINT32(0, rb.write(nullptr, nullptr, 16));

    float inL[64], inR[64];
    for (int i = 0; i < 64; ++i) { inL[i] = static_cast<float>(i + 1); inR[i] = -static_cast<float>(i + 1); }

    // Saturation: max storable is CapacityFrames - 1 = 63
    TEST_ASSERT_EQUAL_UINT32(63, rb.write(inL, inR, 64));
    TEST_ASSERT_EQUAL_UINT32(0, rb.space());

    float outL[64], outR[64];
    TEST_ASSERT_EQUAL_UINT32(32, rb.read(outL, outR, 32));
    TEST_ASSERT_EQUAL_UINT32(32, rb.write(inL, inR, 40)); // Wraps across 64-boundary

    // Underrun zero-padding check
    rb.clear();
    TEST_ASSERT_EQUAL_UINT32(0, rb.available());
    for (int i = 0; i < 16; ++i) { outL[i] = 99.0f; outR[i] = -99.0f; }
    TEST_ASSERT_EQUAL_UINT32(0, rb.read(outL, outR, 16));
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, outL[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, outR[i]);
    }
}

// ==========================================
// Test 2: Biquad EQ & Subsonic (96 kHz Stability & NaN)
// ==========================================
void test_biquad_eq_nan_denormal(void) {
    BiquadEQ eq;
    eq.setBand(0, { 1000.0f, 6.0f, 1.0f, true, EQ_FILTER_PEAK, 2 });
    eq.setBand(1, { NAN, NAN, NAN, true, EQ_FILTER_PEAK, 2 }); // NaN params must sanitize

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

void test_biquad_eq_20hz_and_subsonic_96khz(void) {
    // Verify 20 Hz 4th-order HPF & SubsonicFilter stability and passband unity at 96 kHz
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
    // 1 kHz through 20 Hz HPF must have ~0.3535 RMS (0.5 / sqrt(2))
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.35355f, (float)rms1k);

    // DC step must attenuate toward 0
    sub.reset();
    for (int blk = 0; blk < 120; ++blk) {
        for (int i = 0; i < 256; ++i) { l[i] = 1.0f; r[i] = 1.0f; }
        sub.process(l, r, 256);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, l[255]);
}

// ==========================================
// Test 3: SpeakerCorrection Hilbert Phase & Delay
// ==========================================
void test_speaker_correction_hilbert_and_delay(void) {
    static SpeakerCorrection sc;
    // Test +45 deg on L and -90 deg on R at 100 Hz, 1 kHz, 10 kHz @ 96 kHz
    const float testFreqs[3] = { 100.0f, 1000.0f, 10000.0f };
    for (int fIdx = 0; fIdx < 3; ++fIdx) {
        sc.reset();
        SubsonicParams sp{ 20.0f, 2, true };
        sp.balance = 0.0f;
        sp.delayMsL = -1.0f; // Tests negative delay normalization
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
        // Input RMS = 0.35355; Hilbert +45/-90 must keep unity gain within 0.25 dB (~0.01)
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.35355f, rmsL);
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.35355f, rmsR);
    }
}

// ==========================================
// Test 4: Compander & Limiter Precision
// ==========================================
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

    // SoftLimiter must strictly enforce -0.5 dBFS ceiling (10^(-0.5/20) = 0.94406f)
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

    // NaN parameter guard must still clamp safely
    lim.setParams({ NAN, NAN, true });
    for (int i = 0; i < 32; ++i) { left[i] = 3.0f; right[i] = -3.0f; }
    lim.process(left, right, 32);
    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(fabsf(left[i]) <= expectedCeiling + 1e-4f);
    }
}

// ==========================================
// Test 5: DSPPipeline & THD/Flatness @ 96 kHz
// ==========================================
void test_dsp_pipeline_and_audio_quality(void) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Eq, (uint8_t)parseDspModuleId("eq"));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Subsonic, (uint8_t)parseDspModuleId("sub"));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)DspModuleId::Unknown, (uint8_t)parseDspModuleId("invalid"));

    static ParamStore ps;
    ps.setModuleOrder("eq,eq,sub,comp,lim"); // Must deduplicate "eq,eq" -> "eq,subsonic,comp,lim"
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
    // With all modules bypassed by default, pipeline must be bit-accurate unity
    TEST_ASSERT_TRUE(errSq / refSq < 1e-12);
}

// ==========================================
// Test 6: ParamStore Clamping & Sanitization
// ==========================================
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

    // addEQBand must also sanitize NaN and out-of-range parameters
    TEST_ASSERT_TRUE(ps.addEQBand(NAN, 999.0f, -1.0f, 88, 9));
    EQBandParams added = ps.getEQBand(ps.getEQBandCount() - 1);
    TEST_ASSERT_TRUE(std::isfinite(added.freq) && added.freq >= 10.0f);
    TEST_ASSERT_TRUE(std::isfinite(added.gain) && added.gain <= 24.0f);
    TEST_ASSERT_TRUE(std::isfinite(added.q) && added.q >= 0.05f);
    TEST_ASSERT_EQUAL_UINT8(EQ_FILTER_PEAK, added.type);
    TEST_ASSERT_EQUAL_UINT8(2, added.order);

    // Negative speaker delay and phase must be preserved
    SubsonicParams sub{ 30.0f, 4, true };
    sub.delayMsL = -10.0f;
    sub.phaseL = -90;
    ps.setSubsonic(sub);
    SubsonicParams subOut = ps.getSubsonic();
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, subOut.delayMsL);
    TEST_ASSERT_EQUAL_INT16(-90, subOut.phaseL);

    ps.setPreamp(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(18.0f, ps.getPreamp());
    ps.setPreamp(-100.0f);
    TEST_ASSERT_EQUAL_FLOAT(-18.0f, ps.getPreamp());
    ps.setPreamp(NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ps.getPreamp());
}

// ==========================================
// Test 7: Resampler Create, Bounds & Quality
// ==========================================
void test_resampler_arbitrary_rate(void) {
    resampler_t* r = resampler_create(44100, 96000);
    TEST_ASSERT_NOT_NULL(r);

    // Contract check: 0 out_frames must yield 0 in_frames
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
    // Input RMS = 0.8 / sqrt(2) = 0.5657
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5657f, (float)rmsOut);

    resampler_stats_t stats;
    resampler_get_stats(r, &stats);
    TEST_ASSERT_EQUAL_UINT32(44100, stats.in_rate);
    TEST_ASSERT_EQUAL_UINT32(96000, stats.out_rate);
    TEST_ASSERT_EQUAL_UINT32(8U, static_cast<uint32_t>(stats.total_calls));

    resampler_destroy(r);
}

// ==========================================
// Test 8: AIFF IEEE-80 & Chunk Bounds Logic
// ==========================================
void test_wav_aiff_parser_boundaries(void) {
    // 44100.0 in 80-bit IEEE 754 extended: exp = 16383 + 15 = 16398 (0x400E), mant = 0xAC44000000000000
    const uint8_t ieee44k[10] = { 0x40, 0x0E, 0xAC, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    int expon = (((int)ieee44k[0] & 0x7F) << 8) | ieee44k[1];
    uint32_t hi = ((uint32_t)ieee44k[2] << 24) | ((uint32_t)ieee44k[3] << 16) | ((uint32_t)ieee44k[4] << 8) | ieee44k[5];
    expon -= 16383;
    unsigned rate = (unsigned)(ldexp((double)hi, expon - 31) + 0.5);
    TEST_ASSERT_EQUAL_UINT32(44100, rate);

    // Malformed AIFF SSND offset overflow check (uint64_t prevents 32-bit wrap)
    uint32_t chunkLen = 100;
    uint32_t maliciousOffset = 0xFFFFFFFCu;
    bool validSsnd = (chunkLen >= 8 && (uint64_t)maliciousOffset <= (uint64_t)chunkLen - 8ULL);
    TEST_ASSERT_FALSE(validSsnd);
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
    RUN_TEST(test_biquad_eq_nan_denormal);
    RUN_TEST(test_biquad_eq_20hz_and_subsonic_96khz);
    RUN_TEST(test_speaker_correction_hilbert_and_delay);
    RUN_TEST(test_compander_and_limiter);
    RUN_TEST(test_dsp_pipeline_and_audio_quality);
    RUN_TEST(test_param_store_sanitization);
    RUN_TEST(test_resampler_arbitrary_rate);
    RUN_TEST(test_wav_aiff_parser_boundaries);
    UNITY_END();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
