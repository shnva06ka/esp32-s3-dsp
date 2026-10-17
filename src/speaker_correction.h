/**
 * @file speaker_correction.h
 * @brief Speakers Correction Module (Subsonic Filter + Channel Correction).
 *
 * Runs as the final step of the DSP pipeline.
 * Submodule 1: Subsonic Filter (1st to 4th order high-pass, 20-100 Hz).
 * Submodule 2: Channel Correction:
 *   - Balance (L/R -100 to +100%: left louder on negative, right louder on positive, center 0 neutral)
 *   - Time Shift (per-channel delay in ms, 0 to 20 ms, sub-millisecond interpolated)
 *   - Phase Shift (per-channel 0, 45, 90, 135, 180 degrees via wideband Hilbert all-pass network)
 */
#pragma once

#include <cmath>
#include <cstring>
#include "audio_config.h"
#include "param_store.h"
#include "subsonic.h"

class SpeakerCorrection {
public:
    static constexpr size_t kDelayBufSize = 2048; // Max 20ms @ 48kHz = 960 samples

    SpeakerCorrection() {
        reset();
        SubsonicParams defaultParams;
        setParams(defaultParams);
    }

    void reset() {
        _subsonic.reset();
        memset(_delayBufL, 0, sizeof(_delayBufL));
        memset(_delayBufR, 0, sizeof(_delayBufR));
        _delayWriteIdx = 0;
        memset(_hStateL, 0, sizeof(_hStateL));
        memset(_hStateR, 0, sizeof(_hStateR));
    }

    void setParams(const SubsonicParams& p) {
        _enabled = p.enabled;
        _subsonic.setParams(p);

        // 1. Balance: -100 to +100
        // "to the left makes L louder, to the right makes R louder, in middle does nothing."
        float bal = p.balance;
        if (bal < -100.0f) bal = -100.0f;
        if (bal >  100.0f) bal =  100.0f;

        if (bal <= 0.0f) {
            // Turning left: Left gets louder (up to +6 dB at -100), Right attenuates (down to -24 dB at -100)
            float lDb = (-bal / 100.0f) * 6.0f;
            float rDb = (bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        } else {
            // Turning right: Right gets louder (up to +6 dB at +100), Left attenuates (down to -24 dB at +100)
            float rDb = (bal / 100.0f) * 6.0f;
            float lDb = (-bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        }

        // 2. Time delay: support -20.0 to +20.0 ms
        constexpr float Fs = static_cast<float>(kSampleRate);
        float dMsL = (p.delayMsL < -20.0f) ? -20.0f : ((p.delayMsL > 20.0f) ? 20.0f : p.delayMsL);
        float dMsR = (p.delayMsR < -20.0f) ? -20.0f : ((p.delayMsR > 20.0f) ? 20.0f : p.delayMsR);

        // Relative delay normalization: negative delay on one channel advances it by delaying the other channel
        float minD = (dMsL < dMsR) ? dMsL : dMsR;
        float effL = (minD < 0.0f) ? (dMsL - minD) : dMsL;
        float effR = (minD < 0.0f) ? (dMsR - minD) : dMsR;

        _delaySamplesL = effL * (Fs / 1000.0f);
        _delaySamplesR = effR * (Fs / 1000.0f);

        // 3. Phase shift (quantized to -180, -135, -90, -45, 0, 45, 90, 135, 180 degrees)
        _phaseDegL = _quantizePhase(p.phaseL);
        _phaseDegR = _quantizePhase(p.phaseR);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        // Submodule 1: Subsonic Filter
        _subsonic.process(left, right, frames);

        // Submodule 2: Channel Correction
        const bool needDelayL = (_delaySamplesL > 0.01f);
        const bool needDelayR = (_delaySamplesR > 0.01f);

        for (size_t i = 0; i < frames; ++i) {
            float sL = left[i];
            float sR = right[i];

            // A. Balance
            sL *= _gainL;
            sR *= _gainR;

            // B. Time Delay Buffer
            _delayBufL[_delayWriteIdx] = sL;
            _delayBufR[_delayWriteIdx] = sR;

            if (needDelayL) {
                float rPos = static_cast<float>(_delayWriteIdx) - _delaySamplesL;
                while (rPos < 0.0f) rPos += static_cast<float>(kDelayBufSize);
                int idx0 = static_cast<int>(rPos) & (kDelayBufSize - 1);
                int idx1 = (idx0 + 1) & (kDelayBufSize - 1);
                float frac = rPos - floorf(rPos);
                sL = _delayBufL[idx0] * (1.0f - frac) + _delayBufL[idx1] * frac;
            }

            if (needDelayR) {
                float rPos = static_cast<float>(_delayWriteIdx) - _delaySamplesR;
                while (rPos < 0.0f) rPos += static_cast<float>(kDelayBufSize);
                int idx0 = static_cast<int>(rPos) & (kDelayBufSize - 1);
                int idx1 = (idx0 + 1) & (kDelayBufSize - 1);
                float frac = rPos - floorf(rPos);
                sR = _delayBufR[idx0] * (1.0f - frac) + _delayBufR[idx1] * frac;
            }

            _delayWriteIdx = (_delayWriteIdx + 1) & (kDelayBufSize - 1);

            // C. Phase Shift (-180 to +180)
            left[i]  = _applyPhase(sL, _phaseDegL, _hStateL);
            right[i] = _applyPhase(sR, _phaseDegR, _hStateR);
        }
    }

private:
    SubsonicFilter _subsonic;
    bool           _enabled{false};
    float          _gainL{1.0f};
    float          _gainR{1.0f};
    float          _delaySamplesL{0.0f};
    float          _delaySamplesR{0.0f};
    int16_t        _phaseDegL{0};
    int16_t        _phaseDegR{0};

    // Circular delay buffer
    float          _delayBufL[kDelayBufSize]{};
    float          _delayBufR[kDelayBufSize]{};
    size_t         _delayWriteIdx{0};

    // Hilbert 4-pole all-pass filter state [s0, s1, s2, s3] per channel
    float          _hStateL[4]{};
    float          _hStateR[4]{};

    static inline int16_t _quantizePhase(int16_t deg) {
        if (deg < -157) return -180;
        if (deg < -112) return -135;
        if (deg < -67)  return -90;
        if (deg < -22)  return -45;
        if (deg <= 22)  return 0;
        if (deg <= 67)  return 45;
        if (deg <= 112) return 90;
        if (deg <= 157) return 135;
        return 180;
    }

    static inline float _ap1(float x, float c, float& s) {
        float y = c * x + s;
        s = x - c * y;
        return y;
    }

    static inline float _applyPhase(float x, int16_t deg, float* state) {
        if (deg == 0) return x;
        if (deg == 180 || deg == -180) return -x;

        // Path I (2-pole allpass cascade): c0 = 0.161758, c1 = 0.733029
        float xI = _ap1(_ap1(x, 0.161758f, state[0]), 0.733029f, state[1]);

        // Path Q (2-pole allpass cascade, 90 deg relative to I): c2 = 0.460144, c3 = 0.902267
        float xQ = _ap1(_ap1(x, 0.460144f, state[2]), 0.902267f, state[3]);

        if (deg == 90)   return -xQ;
        if (deg == -90)  return  xQ;
        if (deg == 45)   return  0.70710678f * xI - 0.70710678f * xQ;
        if (deg == -45)  return  0.70710678f * xI + 0.70710678f * xQ;
        if (deg == 135)  return -0.70710678f * xI - 0.70710678f * xQ;
        if (deg == -135) return -0.70710678f * xI + 0.70710678f * xQ;

        return x;
    }
};

