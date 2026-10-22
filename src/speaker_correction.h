// Speakers correction: subsonic high-pass, balance, per-channel delay and phase shift.
#pragma once

#include <cmath>
#include <cstring>
#include "audio_config.h"
#include "param_store.h"
#include "subsonic.h"

class SpeakerCorrection {
public:
    static constexpr size_t kDelayBufSize = 2048; // power of 2, 20 ms is 960 samples

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

        // balance: louder side up to +6 dB, other side down to -24 dB
        float bal = p.balance;
        if (bal < -100.0f) bal = -100.0f;
        if (bal >  100.0f) bal =  100.0f;

        if (bal <= 0.0f) {
            float lDb = (-bal / 100.0f) * 6.0f;
            float rDb = (bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        } else {
            float rDb = (bal / 100.0f) * 6.0f;
            float lDb = (-bal / 100.0f) * 24.0f;
            _gainL = powf(10.0f, lDb / 20.0f);
            _gainR = powf(10.0f, rDb / 20.0f);
        }

        constexpr float Fs = static_cast<float>(kSampleRate);
        float dMsL = (p.delayMsL < -20.0f) ? -20.0f : ((p.delayMsL > 20.0f) ? 20.0f : p.delayMsL);
        float dMsR = (p.delayMsR < -20.0f) ? -20.0f : ((p.delayMsR > 20.0f) ? 20.0f : p.delayMsR);

        // negative delay = delay the other channel instead
        float minD = (dMsL < dMsR) ? dMsL : dMsR;
        float effL = (minD < 0.0f) ? (dMsL - minD) : dMsL;
        float effR = (minD < 0.0f) ? (dMsR - minD) : dMsR;

        _delaySamplesL = effL * (Fs / 1000.0f);
        _delaySamplesR = effR * (Fs / 1000.0f);

        // phase snaps to 45 degree steps
        _phaseDegL = _quantizePhase(p.phaseL);
        _phaseDegR = _quantizePhase(p.phaseR);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        _subsonic.process(left, right, frames);

        const bool needDelayL = (_delaySamplesL > 0.01f);
        const bool needDelayR = (_delaySamplesR > 0.01f);

        for (size_t i = 0; i < frames; ++i) {
            float sL = left[i];
            float sR = right[i];

            sL *= _gainL;
            sR *= _gainR;

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

    float          _delayBufL[kDelayBufSize]{};
    float          _delayBufR[kDelayBufSize]{};
    size_t         _delayWriteIdx{0};

    // allpass state for the phase shifter
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

        // I and Q paths are allpass pairs about 90 degrees apart
        float xI = _ap1(_ap1(x, 0.161758f, state[0]), 0.733029f, state[1]);

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

