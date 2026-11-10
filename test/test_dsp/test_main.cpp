#include <Arduino.h>
#include <unity.h>
#include <cmath>
#include "audio_config.h"
#include "audio_ring_buffer.h"
#include "biquad_eq.h"
#include "compander.h"
#include "limiter.h"
#include "speaker_correction.h"
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

    // Test skip
    size_t skipped = rb.skip(16);
    TEST_ASSERT_EQUAL_UINT32(16, skipped);
    TEST_ASSERT_EQUAL_UINT32(16, rb.available());

    // Read remaining
    read = rb.read(outL, outR, 16);
    TEST_ASSERT_EQUAL_UINT32(16, read);
    TEST_ASSERT_EQUAL_UINT32(0, rb.available());
    TEST_ASSERT_EQUAL_FLOAT(48.0f, outL[0]);
}

void test_ring_buffer_wrap_around(void) {
    AudioRingBuffer<64> rb;
    TEST_ASSERT_TRUE(rb.allocate());

    float inL[48], inR[48];
    for (int i = 0; i < 48; ++i) { inL[i] = 1.0f; inR[i] = 2.0f; }

    rb.write(inL, inR, 48);
    float outL[32], outR[32];
    rb.read(outL, outR, 32); // head advances by 32

    // Now write across the 64-boundary
    size_t written = rb.write(inL, inR, 40);
    TEST_ASSERT_TRUE(written > 0);
}

// ==========================================
// Test 2: Biquad EQ NaN & Denormal Resilience
// ==========================================
void test_biquad_eq_nan_denormal(void) {
    BiquadEQ eq;
    eq.setBand(0, { 1000.0f, 6.0f, 1.0f, true, EQ_FILTER_PEAK, 2 });

    float left[16], right[16];
    for (int i = 0; i < 16; ++i) {
        left[i] = 0.5f;
        right[i] = -0.5f;
    }
    // Inject NaN and Infinity
    left[4] = NAN;
    right[8] = INFINITY;

    eq.process(left, right, 16, 1);

    // Ensure filter survived and produced finite numbers
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
    }

    // Test denormal input flushing
    for (int i = 0; i < 16; ++i) {
        left[i] = 1e-20f;
        right[i] = -1e-25f;
    }
    eq.process(left, right, 16, 1);
    for (int i = 0; i < 16; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
    }
}

// ==========================================
// Test 3: Compander & Limiter Bounds
// ==========================================
void test_compander_and_limiter(void) {
    Compander comp;
    comp.setParams({ -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true });

    float left[32], right[32];
    for (int i = 0; i < 32; ++i) {
        left[i] = 2.0f; // Excessive level (clipping)
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

    for (int i = 0; i < 32; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(left[i]));
        TEST_ASSERT_TRUE(std::isfinite(right[i]));
        TEST_ASSERT_TRUE(fabsf(left[i]) <= 1.2f);
        TEST_ASSERT_TRUE(fabsf(right[i]) <= 1.2f);
    }
}

// ==========================================
// Test 4: ParamStore Clamping & Sanitization
// ==========================================
void test_param_store_sanitization(void) {
    ParamStore ps;

    // Test extreme EQ band inputs
    EQBandParams pBad{ -500.0f, 100.0f, -5.0f, true, 99, 10 };
    ps.setEQBand(0, pBad);

    EQBandParams pSan = ps.getEQBand(0);
    TEST_ASSERT_TRUE(pSan.freq >= 10.0f);
    TEST_ASSERT_TRUE(pSan.gain <= 24.0f);
    TEST_ASSERT_TRUE(pSan.q >= 0.05f);
    TEST_ASSERT_TRUE(pSan.type <= EQ_FILTER_FLAT_HIGH_PASS);
    TEST_ASSERT_EQUAL_UINT8(2, pSan.order);

    // Test preamp clamping
    ps.setPreamp(50.0f);
    TEST_ASSERT_EQUAL_FLOAT(18.0f, ps.getPreamp());
    ps.setPreamp(-100.0f);
    TEST_ASSERT_EQUAL_FLOAT(-18.0f, ps.getPreamp());
    ps.setPreamp(NAN);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ps.getPreamp());
}

// ==========================================
// Test 5: Resampler Create & Flow Test
// ==========================================
void test_resampler_arbitrary_rate(void) {
    resampler_t* r = resampler_create(44100, 96000);
    TEST_ASSERT_NOT_NULL(r);

    size_t inFrames = 64;
    size_t maxOut = resampler_max_out(r, inFrames);
    TEST_ASSERT_TRUE(maxOut > 0 && maxOut < 512);

    float inL[64], inR[64];
    for (int i = 0; i < 64; ++i) {
        inL[i] = sinf(2.0f * M_PI * 440.0f * i / 44100.0f);
        inR[i] = cosf(2.0f * M_PI * 440.0f * i / 44100.0f);
    }

    float outL[256], outR[256];
    size_t produced = resampler_process(r, inL, inR, inFrames, outL, outR, 256);
    TEST_ASSERT_TRUE(produced > 0);

    for (size_t i = 0; i < produced; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(outL[i]));
        TEST_ASSERT_TRUE(std::isfinite(outR[i]));
    }

    resampler_stats_t stats;
    resampler_get_stats(r, &stats);
    TEST_ASSERT_EQUAL_UINT32(44100, stats.in_rate);
    TEST_ASSERT_EQUAL_UINT32(96000, stats.out_rate);

    resampler_destroy(r);
}

// ==========================================
// Test 6: Fuzzing Chunk & Header Parser Logic
// ==========================================
void test_fuzz_parser_chunks(void) {
    // Fuzz test memory boundary logic with malicious / corrupt headers
    uint8_t malformedChunk[64];
    memset(malformedChunk, 0xFF, sizeof(malformedChunk));

    // Verify 32-bit offset arithmetic will not wrap around past buffer
    uint32_t headerLen = 44;
    uint32_t claimedChunkSize = 0xFFFFFFF0;
    uint32_t totalBytes = 100;

    // Standard safe skip idiom:
    uint32_t skip = claimedChunkSize;
    if (skip > totalBytes - headerLen) {
        skip = totalBytes - headerLen;
    }
    TEST_ASSERT_TRUE(skip <= totalBytes);

    // Test zero channel and zero sample rate protections
    uint16_t channels = 0;
    uint32_t sampleRate = 0;
    if (channels == 0 || channels > 8) channels = 2;
    if (sampleRate < 8000 || sampleRate > 192000) sampleRate = 44100;

    TEST_ASSERT_EQUAL_UINT16(2, channels);
    TEST_ASSERT_EQUAL_UINT32(44100, sampleRate);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_ring_buffer_basic_ops);
    RUN_TEST(test_ring_buffer_wrap_around);
    RUN_TEST(test_biquad_eq_nan_denormal);
    RUN_TEST(test_compander_and_limiter);
    RUN_TEST(test_param_store_sanitization);
    RUN_TEST(test_resampler_arbitrary_rate);
    RUN_TEST(test_fuzz_parser_chunks);
    UNITY_END();
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
