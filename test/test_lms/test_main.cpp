#include <unity.h>
#include <cstdint>
#include <cstring>
#include <vector>

#include "../../src/lms_core.h"
#include "../../src/audio_config.h"
#include "../../src/dsp_pipeline.h"
#include <cmath>

static const uint8_t kFixHelo[71] = {0x48, 0x45, 0x4C, 0x4F, 0x00, 0x00, 0x00, 0x3F, 0x0C, 0x00, 0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x6E, 0x66, 0x6C, 0x63, 0x2C, 0x70, 0x63, 0x6D, 0x2C, 0x4D, 0x61, 0x78, 0x53, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x52, 0x61, 0x74, 0x65, 0x3D, 0x39, 0x36, 0x30, 0x30, 0x30};
static const uint8_t kFixStat[61] = {0x53, 0x54, 0x41, 0x54, 0x00, 0x00, 0x00, 0x35, 0x53, 0x54, 0x4D, 0x74, 0x00, 0x6D, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x1C, 0xBE, 0x99, 0x1A, 0x14, 0x00, 0x46, 0x00, 0x00, 0x13, 0x88, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x05, 0xDC, 0xCA, 0xFE, 0xBA, 0xBE, 0x00, 0x00};
static const uint8_t kFixStrm[84] = {0x00, 0x52, 0x73, 0x74, 0x72, 0x6D, 0x73, 0x31, 0x66, 0x3F, 0x3F, 0x3F, 0x3F, 0xFF, 0x00, 0x00, 0x30, 0x41, 0x14, 0x00, 0x00, 0x01, 0x80, 0x00, 0x23, 0x28, 0x00, 0x00, 0x00, 0x00, 0x47, 0x45, 0x54, 0x20, 0x2F, 0x73, 0x74, 0x72, 0x65, 0x61, 0x6D, 0x2E, 0x66, 0x6C, 0x61, 0x63, 0x3F, 0x70, 0x6C, 0x61, 0x79, 0x65, 0x72, 0x3D, 0x30, 0x32, 0x3A, 0x31, 0x31, 0x3A, 0x32, 0x32, 0x3A, 0x33, 0x33, 0x3A, 0x34, 0x34, 0x3A, 0x35, 0x35, 0x20, 0x48, 0x54, 0x54, 0x50, 0x2F, 0x31, 0x2E, 0x30, 0x0D, 0x0A, 0x0D, 0x0A};
static const uint8_t kFixAudg[24] = {0x00, 0x16, 0x61, 0x75, 0x64, 0x67, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x01, 0xFF, 0x00, 0x00, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00};

#define ASSERT_CHAR(e, a) TEST_ASSERT_EQUAL_UINT8((uint8_t)(e), (uint8_t)(a))

void setUp(void) {}
void tearDown(void) {}

void test_helo_matches_fixture(void) {
    const uint8_t mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
    uint8_t out[128];
    const size_t n = lms::buildHelo(out, sizeof(out), mac, 12, 0, 0, "flc,pcm,MaxSampleRate=96000");
    TEST_ASSERT_EQUAL_UINT32(sizeof(kFixHelo), n);
    TEST_ASSERT_EQUAL_MEMORY(kFixHelo, out, n);
}

void test_helo_rejects_small_buffer(void) {
    const uint8_t mac[6] = {0};
    uint8_t out[40];
    TEST_ASSERT_EQUAL_UINT32(0, lms::buildHelo(out, sizeof(out), mac, 12, 0, 0, "flc"));
}

void test_stat_matches_fixture(void) {
    lms::StatInfo s;
    s.streamBufSize = 2097152;  s.streamBufFullness = 1000;
    s.bytesReceived = 123456789012ULL;
    s.signal = 70;              s.jiffies = 5000;
    s.outBufSize = 786432;      s.outBufFullness = 4096;
    s.elapsedSeconds = 12;      s.elapsedMs = 1500;
    s.serverTimestamp = 0xCAFEBABE;
    uint8_t out[80];
    const size_t n = lms::buildStat(out, sizeof(out), "STMt", s);
    TEST_ASSERT_EQUAL_UINT32(sizeof(kFixStat), n);
    TEST_ASSERT_EQUAL_MEMORY(kFixStat, out, n);
}

void test_parse_strm_frame(void) {
    lms::Frame f; size_t used = 0;
    TEST_ASSERT_EQUAL(lms::FrameResult::Ok, lms::parseFrame(kFixStrm, sizeof(kFixStrm), f, used));
    TEST_ASSERT_EQUAL_UINT32(sizeof(kFixStrm), used);
    TEST_ASSERT_TRUE(lms::opIs(f, "strm"));
    lms::Strm s;
    TEST_ASSERT_TRUE(lms::parseStrm(f.data, f.dataLen, s));
    ASSERT_CHAR('s', s.command);
    ASSERT_CHAR('1', s.autostart);
    ASSERT_CHAR('f', s.format);
    ASSERT_CHAR('?', s.pcmRate);
    TEST_ASSERT_EQUAL_UINT8(255, s.threshold);
    TEST_ASSERT_EQUAL_UINT8(0x41, s.flags);
    TEST_ASSERT_EQUAL_UINT8(20, s.outputThreshold);
    TEST_ASSERT_EQUAL_UINT32(0x00018000, s.replayGain);
    TEST_ASSERT_EQUAL_UINT16(9000, s.port);
    TEST_ASSERT_EQUAL_UINT32(0, s.ip);
    const char* http = "GET /stream.flac?player=02:11:22:33:44:55 HTTP/1.0\r\n\r\n";
    TEST_ASSERT_EQUAL_UINT32(std::strlen(http), s.httpLen);
    TEST_ASSERT_EQUAL_MEMORY(http, s.http, s.httpLen);
}

void test_parse_strm_rejects_short(void) {
    uint8_t d[10] = {0};
    lms::Strm s;
    TEST_ASSERT_FALSE(lms::parseStrm(d, sizeof(d), s));
}

void test_parse_audg(void) {
    lms::Frame f; size_t used = 0;
    TEST_ASSERT_EQUAL(lms::FrameResult::Ok, lms::parseFrame(kFixAudg, sizeof(kFixAudg), f, used));
    TEST_ASSERT_TRUE(lms::opIs(f, "audg"));
    lms::Audg a;
    TEST_ASSERT_TRUE(lms::parseAudg(f.data, f.dataLen, a));
    TEST_ASSERT_EQUAL_UINT32(0x8000, a.left);
    TEST_ASSERT_EQUAL_UINT32(0x10000, a.right);
}

void test_frame_partial_then_complete(void) {
    lms::Frame f; size_t used = 99;
    TEST_ASSERT_EQUAL(lms::FrameResult::NeedMore, lms::parseFrame(kFixAudg, 1, f, used));
    TEST_ASSERT_EQUAL_UINT32(0, used);
    TEST_ASSERT_EQUAL(lms::FrameResult::NeedMore, lms::parseFrame(kFixAudg, sizeof(kFixAudg) - 1, f, used));
    TEST_ASSERT_EQUAL(lms::FrameResult::Ok, lms::parseFrame(kFixAudg, sizeof(kFixAudg), f, used));
}

void test_frame_two_back_to_back(void) {
    std::vector<uint8_t> buf(kFixAudg, kFixAudg + sizeof(kFixAudg));
    buf.insert(buf.end(), kFixStrm, kFixStrm + sizeof(kFixStrm));
    lms::Frame f; size_t used = 0;
    TEST_ASSERT_EQUAL(lms::FrameResult::Ok, lms::parseFrame(buf.data(), buf.size(), f, used));
    TEST_ASSERT_TRUE(lms::opIs(f, "audg"));
    TEST_ASSERT_EQUAL(lms::FrameResult::Ok, lms::parseFrame(buf.data() + used, buf.size() - used, f, used));
    TEST_ASSERT_TRUE(lms::opIs(f, "strm"));
}

void test_frame_invalid_length(void) {
    const uint8_t bad[6] = {0x00, 0x02, 'a', 'b', 'c', 'd'};
    lms::Frame f; size_t used = 0;
    TEST_ASSERT_EQUAL(lms::FrameResult::Invalid, lms::parseFrame(bad, sizeof(bad), f, used));
}

void test_field_helpers(void) {
    TEST_ASSERT_EQUAL_UINT32(44100, lms::pcmRateFromCode('3'));
    TEST_ASSERT_EQUAL_UINT32(96000, lms::pcmRateFromCode('9'));
    TEST_ASSERT_EQUAL_UINT32(0, lms::pcmRateFromCode('?'));
    TEST_ASSERT_EQUAL_UINT32(16, lms::pcmBitsFromCode('1'));
    TEST_ASSERT_EQUAL_UINT32(0, lms::pcmBitsFromCode('?'));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, lms::fixed16ToFloat(0x8000));
}

void test_volume_gains(void) {
    lms::Gains g = lms::volumeGains(0x8000, 0x10000, true);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, g.left);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, g.right);
    g = lms::volumeGains(0x8000, 0x10000, false);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, g.right);
    g = lms::volumeGains(0xFFFFFFFFu, 0, true);
    TEST_ASSERT_EQUAL_FLOAT(lms::kMaxVolumeGain, g.left);
}

void test_track_gains(void) {
    lms::Gains g = lms::trackGains(0, 0);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, g.left);
    g = lms::trackGains(0x8000, 0x02);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, g.left);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, g.right);
}

static std::vector<uint8_t> makeWav(uint16_t ch, uint32_t rate, uint16_t bits, bool withListChunk) {
    std::vector<uint8_t> v;
    auto s = [&](const char* t) { v.insert(v.end(), t, t + 4); };
    auto u32 = [&](uint32_t x) { for (int i = 0; i < 4; ++i) v.push_back((uint8_t)(x >> (8 * i))); };
    auto u16 = [&](uint16_t x) { v.push_back((uint8_t)x); v.push_back((uint8_t)(x >> 8)); };
    s("RIFF"); u32(0xFFFFFFFFu); s("WAVE");
    s("fmt "); u32(16); u16(1); u16(ch); u32(rate); u32(rate * ch * bits / 8); u16(ch * bits / 8); u16(bits);
    if (withListChunk) { s("LIST"); u32(5); for (int i = 0; i < 5; ++i) v.push_back('x'); v.push_back(0); }
    s("data"); u32(0xFFFFFFFFu);
    return v;
}

void test_wav_header_basic(void) {
    const auto w = makeWav(2, 96000, 24, false);
    lms::PcmInfo info;
    const int r = lms::parsePcmHeader(w.data(), w.size(), info);
    TEST_ASSERT_EQUAL_INT((int)w.size(), r);
    TEST_ASSERT_EQUAL_UINT32(96000, info.rate);
    TEST_ASSERT_EQUAL_UINT8(24, info.bits);
    TEST_ASSERT_EQUAL_UINT8(2, info.channels);
    TEST_ASSERT_FALSE(info.bigEndian);
}

void test_wav_header_skips_extra_chunk(void) {
    const auto w = makeWav(2, 44100, 16, true);
    lms::PcmInfo info;
    TEST_ASSERT_EQUAL_INT((int)w.size(), lms::parsePcmHeader(w.data(), w.size(), info));
    TEST_ASSERT_EQUAL_UINT32(44100, info.rate);
}

void test_wav_header_needs_more_then_ok(void) {
    const auto w = makeWav(2, 48000, 16, false);
    lms::PcmInfo info;
    TEST_ASSERT_EQUAL_INT(lms::kPcmNeedMore, lms::parsePcmHeader(w.data(), 10, info));
    TEST_ASSERT_EQUAL_INT(lms::kPcmNeedMore, lms::parsePcmHeader(w.data(), w.size() - 1, info));
    TEST_ASSERT_EQUAL_INT((int)w.size(), lms::parsePcmHeader(w.data(), w.size(), info));
}

void test_wav_header_rejects_garbage_and_float(void) {
    uint8_t junk[64];
    std::memset(junk, 0x55, sizeof(junk));
    lms::PcmInfo info;
    TEST_ASSERT_EQUAL_INT(lms::kPcmInvalid, lms::parsePcmHeader(junk, sizeof(junk), info));
    auto w = makeWav(2, 48000, 16, false);
    w[20] = 3;
    TEST_ASSERT_EQUAL_INT(lms::kPcmInvalid, lms::parsePcmHeader(w.data(), w.size(), info));
}

void test_aiff_header(void) {
    std::vector<uint8_t> v;
    auto s = [&](const char* t) { v.insert(v.end(), t, t + 4); };
    auto b32 = [&](uint32_t x) { for (int i = 3; i >= 0; --i) v.push_back((uint8_t)(x >> (8 * i))); };
    auto b16 = [&](uint16_t x) { v.push_back((uint8_t)(x >> 8)); v.push_back((uint8_t)x); };
    s("FORM"); b32(0); s("AIFF");
    s("COMM"); b32(18); b16(2); b32(0); b16(16);
    const uint8_t ext44100[10] = {0x40, 0x0E, 0xAC, 0x44, 0, 0, 0, 0, 0, 0};
    v.insert(v.end(), ext44100, ext44100 + 10);
    s("SSND"); b32(8); b32(0); b32(0);
    lms::PcmInfo info;
    TEST_ASSERT_EQUAL_INT((int)v.size(), lms::parsePcmHeader(v.data(), v.size(), info));
    TEST_ASSERT_EQUAL_UINT32(44100, info.rate);
    TEST_ASSERT_EQUAL_UINT8(16, info.bits);
    TEST_ASSERT_TRUE(info.bigEndian);
}

void test_pcm_convert_16_le_stereo(void) {
    lms::PcmInfo info; info.bits = 16; info.channels = 2; info.bigEndian = false;
    const uint8_t src[8] = {0x00, 0x40, 0x00, 0xC0, 0xFF, 0x7F, 0x00, 0x80};
    float L[2], R[2];
    lms::pcmToFloat(info, src, 2, L, R);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, L[0]);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, R[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, L[1]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, R[1]);
}

void test_pcm_convert_24_be_and_mono(void) {
    lms::PcmInfo info; info.bits = 24; info.channels = 1; info.bigEndian = true;
    const uint8_t src[3] = {0x40, 0x00, 0x00};
    float L[1], R[1];
    lms::pcmToFloat(info, src, 1, L, R);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, L[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, R[0]);
}

void test_pcm_convert_8bit_unsigned(void) {
    lms::PcmInfo info; info.bits = 8; info.channels = 1; info.bigEndian = false;
    const uint8_t src[3] = {128, 255, 0};
    float L[3], R[3];
    lms::pcmToFloat(info, src, 3, L, R);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, L[0]);
    TEST_ASSERT_TRUE(L[1] > 0.99f);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, L[2]);
}

void test_byte_ring_wraps(void) {
    uint8_t store[16];
    lms::ByteRing r;
    TEST_ASSERT_TRUE(r.init(store, 16));
    uint8_t in[12], out[12];
    for (int i = 0; i < 12; ++i) in[i] = (uint8_t)i;
    TEST_ASSERT_EQUAL_UINT32(12, r.write(in, 12));
    TEST_ASSERT_EQUAL_UINT32(8, r.read(out, 8));
    TEST_ASSERT_EQUAL_UINT32(12, r.write(in, 12));
    TEST_ASSERT_EQUAL_UINT32(16, r.available());
    TEST_ASSERT_EQUAL_UINT32(0, r.write(in, 1));
    uint8_t all[16];
    TEST_ASSERT_EQUAL_UINT32(16, r.read(all, 16));
    TEST_ASSERT_EQUAL_UINT8(8, all[0]);
    TEST_ASSERT_EQUAL_UINT8(11, all[3]);
    TEST_ASSERT_EQUAL_UINT8(0, all[4]);
    TEST_ASSERT_EQUAL_UINT8(7, all[11]);
}

void test_byte_ring_rejects_non_power_of_two(void) {
    uint8_t store[12];
    lms::ByteRing r;
    TEST_ASSERT_FALSE(r.init(store, 12));
}

static lms::Levels lv(uint32_t streamBytes, uint32_t outFrames, bool done = false, bool eof = false, bool under = false, uint32_t now = 0) {
    lms::Levels l;
    l.streamBytes = streamBytes; l.outFrames = outFrames; l.decoderDone = done;
    l.streamEof = eof; l.underrun = under; l.nowMs = now;
    return l;
}

void test_transport_autostart_flow(void) {
    lms::Transport t;
    TEST_ASSERT_EQUAL_UINT32(lms::EV_c, t.onStart('1', 255, 2, false));
    TEST_ASSERT_EQUAL(lms::Phase::Buffering, t.phase());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(1000, 100)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_s, t.tick(lv(1000, 19200)));
    TEST_ASSERT_TRUE(t.isPlaying());
    TEST_ASSERT_EQUAL_UINT32(lms::EV_d, t.tick(lv(0, 5000, true, true)));
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(0, 5000, true, true)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_u, t.tick(lv(0, 0, true, true)));
    TEST_ASSERT_EQUAL(lms::Phase::Stopped, t.phase());
}

void test_transport_manual_start_waits_for_unpause(void) {
    lms::Transport t;
    t.onStart('0', 10, 0, false);
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(100, 50000)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_l, t.tick(lv(10240, 50000)));
    TEST_ASSERT_EQUAL(lms::Phase::Buffering, t.phase());
    TEST_ASSERT_EQUAL_UINT32(0, t.onUnpause());
    TEST_ASSERT_EQUAL_UINT32(lms::EV_s, t.tick(lv(10240, 50000)));
    TEST_ASSERT_TRUE(t.isPlaying());
}

void test_transport_threshold_reported_on_eof(void) {
    lms::Transport t;
    t.onStart('2', 255, 0, false);
    TEST_ASSERT_EQUAL_UINT32(lms::EV_l, t.tick(lv(500, 0, false, true)));
}

void test_transport_pause_resume(void) {
    lms::Transport t;
    t.onStart('1', 1, 0, false);
    t.tick(lv(5000, 9600));
    TEST_ASSERT_TRUE(t.isPlaying());
    TEST_ASSERT_EQUAL_UINT32(lms::EV_p, t.onPause(0, 1000));
    TEST_ASSERT_TRUE(t.isPaused());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 9600)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_r, t.onUnpause());
    TEST_ASSERT_TRUE(t.isPlaying());
}

void test_transport_timed_pause_resumes_silently(void) {
    lms::Transport t;
    t.onStart('1', 1, 0, false);
    t.tick(lv(5000, 9600));
    TEST_ASSERT_EQUAL_UINT32(0, t.onPause(500, 1000));
    TEST_ASSERT_TRUE(t.isPaused());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 9600, false, false, false, 1400)));
    TEST_ASSERT_TRUE(t.isPaused());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 9600, false, false, false, 1500)));
    TEST_ASSERT_TRUE(t.isPlaying());
}

void test_transport_underrun_rebuffers_without_second_stms(void) {
    lms::Transport t;
    t.onStart('1', 1, 1, false);
    TEST_ASSERT_EQUAL_UINT32(lms::EV_s, t.tick(lv(5000, 9600)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_o, t.tick(lv(5000, 0, false, false, true)));
    TEST_ASSERT_EQUAL(lms::Phase::Buffering, t.phase());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 9600)));
    TEST_ASSERT_TRUE(t.isPlaying());
}

void test_transport_stop_and_flush(void) {
    lms::Transport t;
    t.onStart('1', 1, 0, false);
    TEST_ASSERT_EQUAL_UINT32(lms::EV_f, t.onStop());
    TEST_ASSERT_EQUAL(lms::Phase::Off, t.phase());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 9600)));
    t.onStart('1', 1, 0, false);
    TEST_ASSERT_EQUAL_UINT32(lms::EV_f, t.onFlush());
}

void test_transport_gapless_keeps_playing(void) {
    lms::Transport t;
    t.onStart('1', 1, 0, false);
    t.tick(lv(5000, 9600));
    TEST_ASSERT_TRUE(t.isPlaying());
    TEST_ASSERT_EQUAL_UINT32(lms::EV_c, t.onStart('1', 1, 0, true));
    TEST_ASSERT_TRUE(t.isPlaying());
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 0, false)));
    TEST_ASSERT_EQUAL_UINT32(lms::EV_s, t.onTrackBoundary());
    TEST_ASSERT_EQUAL_UINT32(0, t.onTrackBoundary());
}

void test_transport_status_request_and_error(void) {
    lms::Transport t;
    TEST_ASSERT_EQUAL_UINT32(lms::EV_t, t.onStatusRequest());
    t.onStart('1', 1, 0, false);
    TEST_ASSERT_EQUAL_UINT32(lms::EV_n, t.onStreamError());
    TEST_ASSERT_EQUAL(lms::Phase::Stopped, t.phase());
}

void test_event_codes(void) {
    TEST_ASSERT_EQUAL_STRING("STMc", lms::eventCode(lms::EV_c));
    TEST_ASSERT_EQUAL_STRING("STMu", lms::eventCode(lms::EV_u));
    TEST_ASSERT_EQUAL_STRING("STMt", lms::eventCode(lms::EV_t));
    TEST_ASSERT_TRUE(lms::EV_c < lms::EV_e && lms::EV_e < lms::EV_s);
}

void test_audio_rate_validation(void) {
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(44100));
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(48000));
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(88200));
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(96000));
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(176400));
    TEST_ASSERT_TRUE(audio_rate::isValidDacRate(192000));
    TEST_ASSERT_FALSE(audio_rate::isValidDacRate(0));
    TEST_ASSERT_FALSE(audio_rate::isValidDacRate(32000));
    TEST_ASSERT_FALSE(audio_rate::isValidDacRate(384000));

    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::clampAdcRate(44100));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::clampAdcRate(96000));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::clampAdcRate(192000));
    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::clampAdcRate(0));
    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::clampAdcRate(32000));

    TEST_ASSERT_EQUAL_UINT32(192000, audio_rate::clampDacRate(192000));
    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::clampDacRate(384000));
    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::clampDacRate(0));
}

void test_audio_rate_active_selection(void) {
    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::activeRate(audio_rate::Source::Aux, 44100, 192000, 192000));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::activeRate(audio_rate::Source::Aux, 96000, 44100, 192000));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::activeRate(audio_rate::Source::Aux, 192000, 192000, 44100));

    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::activeRate(audio_rate::Source::Bt, 96000, 192000, 192000));

    TEST_ASSERT_EQUAL_UINT32(44100, audio_rate::activeRate(audio_rate::Source::Wifi, 44100, 44100, 192000));
    TEST_ASSERT_EQUAL_UINT32(192000, audio_rate::activeRate(audio_rate::Source::Wifi, 44100, 192000, 192000));
    TEST_ASSERT_EQUAL_UINT32(88200, audio_rate::activeRate(audio_rate::Source::Wifi, 44100, 96000, 88200));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::activeRate(audio_rate::Source::Wifi, 44100, 96000, 176400));
    TEST_ASSERT_EQUAL_UINT32(96000, audio_rate::activeRate(audio_rate::Source::Wifi, 44100, 96000, 0));

    TEST_ASSERT_EQUAL_UINT32(256, audio_rate::blockFramesForRate(44100));
    TEST_ASSERT_EQUAL_UINT32(256, audio_rate::blockFramesForRate(96000));
    TEST_ASSERT_EQUAL_UINT32(512, audio_rate::blockFramesForRate(176400));
    TEST_ASSERT_EQUAL_UINT32(512, audio_rate::blockFramesForRate(192000));
}

static float measureSineRms(float freqHz, uint32_t fs, IirEQ& eq) {
    const size_t N = 4096;
    std::vector<float> l(N), r(N);
    for (size_t i = 0; i < N; ++i) {
        float s = 0.25f * sinf(2.0f * 3.14159265f * freqHz * static_cast<float>(i) / static_cast<float>(fs));
        l[i] = s;
        r[i] = s;
    }
    eq.process(l.data(), r.data(), N, 1);
    double sum = 0.0;
    for (size_t i = N / 2; i < N; ++i) sum += (double)l[i] * l[i];
    return sqrtf(static_cast<float>(sum / (N / 2)));
}

void test_iir_eq_runtime_sample_rate(void) {
    IirEQ eq;
    eq.setSampleRate(44100);
    eq.setBand(0, EQBandParams(10000.0f, 6.0f, 2.0f, true, EQ_FILTER_PEAK));

    const float baseline = 0.25f / sqrtf(2.0f);
    const float rms44 = measureSineRms(10000.0f, 44100, eq);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, baseline * 2.0f, rms44);

    eq.setSampleRate(192000);
    eq.reset();
    const float rms192 = measureSineRms(10000.0f, 192000, eq);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, baseline * 2.0f, rms192);

    eq.setSampleRate(44100);
    eq.setBand(0, EQBandParams(30000.0f, 6.0f, 2.0f, true, EQ_FILTER_PEAK));
    eq.setSampleRate(192000);
    eq.reset();
    const float rms30k = measureSineRms(30000.0f, 192000, eq);
    TEST_ASSERT_FLOAT_WITHIN(0.06f, baseline * 2.0f, rms30k);
}

void test_subsonic_runtime_sample_rate(void) {
    SubsonicFilter sub;
    sub.setSampleRate(44100);
    sub.setParams(SubsonicParams(80.0f, 2, true));

    auto runSub = [&](uint32_t fs) -> float {
        const size_t N = 8192;
        std::vector<float> l(N), r(N);
        for (size_t i = 0; i < N; ++i) {
            float s = 0.5f * sinf(2.0f * 3.14159265f * 40.0f * static_cast<float>(i) / static_cast<float>(fs));
            l[i] = s;
            r[i] = s;
        }
        sub.process(l.data(), r.data(), N);
        double sum = 0.0;
        for (size_t i = N / 2; i < N; ++i) sum += (double)l[i] * l[i];
        return sqrtf(static_cast<float>(sum / (N / 2)));
    };

    const float rms44 = runSub(44100);
    sub.setSampleRate(192000);
    sub.reset();
    const float rms192 = runSub(192000);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, rms44, rms192);
    TEST_ASSERT_TRUE(rms192 < 0.12f);
}

void test_speaker_correction_delay_at_192k(void) {
    SpeakerCorrection sc;
    sc.setSampleRate(192000);
    SubsonicParams p;
    p.enabled  = true;
    p.freq     = 20.0f;
    p.order    = 2;
    p.delayMsL = 20.0f;
    p.delayMsR = -20.0f;
    sc.setParams(p);

    const size_t N = 8192;
    std::vector<float> l(N, 0.0f), r(N, 0.0f);
    l[0] = 1.0f;
    r[0] = 1.0f;
    sc.process(l.data(), r.data(), N);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, r[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, l[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, l[7680]);
}

void test_compander_runtime_sample_rate(void) {
    Compander comp;
    comp.setSampleRate(192000);
    CompanderParams p;
    p.enabled   = true;
    p.threshold = -18.0f;
    p.ratio     = 4.0f;
    p.attackMs  = 5.0f;
    p.releaseMs = 100.0f;
    comp.setParams(p);

    std::vector<float> l(512, 0.5f), r(512, 0.5f);
    for (int b = 0; b < 10; ++b) {
        std::fill(l.begin(), l.end(), 0.5f);
        std::fill(r.begin(), r.end(), 0.5f);
        comp.process(l.data(), r.data(), 512);
    }
    TEST_ASSERT_TRUE(std::isfinite(l[511]));
    TEST_ASSERT_TRUE(l[511] < 0.45f && l[511] > 0.05f);
}

void test_transport_dynamic_output_rate(void) {
    lms::Transport t;
    t.onStart('1', 1, 10, false, 192000);
    TEST_ASSERT_EQUAL_UINT32(0, t.tick(lv(5000, 100000)));
    TEST_ASSERT_EQUAL(lms::Phase::Buffering, t.phase());
    TEST_ASSERT_EQUAL_UINT32(lms::EV_s, t.tick(lv(5000, 192000)));
    TEST_ASSERT_EQUAL(lms::Phase::Playing, t.phase());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_helo_matches_fixture);
    RUN_TEST(test_helo_rejects_small_buffer);
    RUN_TEST(test_stat_matches_fixture);
    RUN_TEST(test_parse_strm_frame);
    RUN_TEST(test_parse_strm_rejects_short);
    RUN_TEST(test_parse_audg);
    RUN_TEST(test_frame_partial_then_complete);
    RUN_TEST(test_frame_two_back_to_back);
    RUN_TEST(test_frame_invalid_length);
    RUN_TEST(test_field_helpers);
    RUN_TEST(test_volume_gains);
    RUN_TEST(test_track_gains);
    RUN_TEST(test_wav_header_basic);
    RUN_TEST(test_wav_header_skips_extra_chunk);
    RUN_TEST(test_wav_header_needs_more_then_ok);
    RUN_TEST(test_wav_header_rejects_garbage_and_float);
    RUN_TEST(test_aiff_header);
    RUN_TEST(test_pcm_convert_16_le_stereo);
    RUN_TEST(test_pcm_convert_24_be_and_mono);
    RUN_TEST(test_pcm_convert_8bit_unsigned);
    RUN_TEST(test_byte_ring_wraps);
    RUN_TEST(test_byte_ring_rejects_non_power_of_two);
    RUN_TEST(test_transport_autostart_flow);
    RUN_TEST(test_transport_manual_start_waits_for_unpause);
    RUN_TEST(test_transport_threshold_reported_on_eof);
    RUN_TEST(test_transport_pause_resume);
    RUN_TEST(test_transport_timed_pause_resumes_silently);
    RUN_TEST(test_transport_underrun_rebuffers_without_second_stms);
    RUN_TEST(test_transport_stop_and_flush);
    RUN_TEST(test_transport_gapless_keeps_playing);
    RUN_TEST(test_transport_status_request_and_error);
    RUN_TEST(test_event_codes);
    RUN_TEST(test_audio_rate_validation);
    RUN_TEST(test_audio_rate_active_selection);
    RUN_TEST(test_iir_eq_runtime_sample_rate);
    RUN_TEST(test_subsonic_runtime_sample_rate);
    RUN_TEST(test_speaker_correction_delay_at_192k);
    RUN_TEST(test_compander_runtime_sample_rate);
    RUN_TEST(test_transport_dynamic_output_rate);
    return UNITY_END();
}
