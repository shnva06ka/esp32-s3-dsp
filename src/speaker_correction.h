#pragma once

#include <cmath>
#include <cstring>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#ifndef HAVE_ESP_DSP
#define HAVE_ESP_DSP 1
#endif
#endif
#include "audio_config.h"
#include "param_store.h"
#include "subsonic.h"

class SpeakerCorrection {
public:
    static constexpr size_t kDelayBufSize = 4096;
    static_assert((kDelayBufSize & (kDelayBufSize - 1)) == 0, "kDelayBufSize must be a power of 2");

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

        float bal = p.balance;
        if (!std::isfinite(bal) || bal < -100.0f) bal = -100.0f;
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
        if (!std::isfinite(dMsL)) dMsL = 0.0f;
        if (!std::isfinite(dMsR)) dMsR = 0.0f;

        float minD = (dMsL < dMsR) ? dMsL : dMsR;
        float effL = (minD < 0.0f) ? (dMsL - minD) : dMsL;
        float effR = (minD < 0.0f) ? (dMsR - minD) : dMsR;

        const bool hadDelayL = (_delaySamplesL > 0.01f);
        const bool hadDelayR = (_delaySamplesR > 0.01f);

        _delaySamplesL = effL * (Fs / 1000.0f);
        _delaySamplesR = effR * (Fs / 1000.0f);

        constexpr float maxDelaySamples = static_cast<float>(kDelayBufSize - 2);
        if (_delaySamplesL > maxDelaySamples) _delaySamplesL = maxDelaySamples;
        if (_delaySamplesR > maxDelaySamples) _delaySamplesR = maxDelaySamples;

        const bool hasDelayL = (_delaySamplesL > 0.01f);
        const bool hasDelayR = (_delaySamplesR > 0.01f);
        if (!hadDelayL && hasDelayL) {
            memset(_delayBufL, 0, sizeof(_delayBufL));
        }
        if (!hadDelayR && hasDelayR) {
            memset(_delayBufR, 0, sizeof(_delayBufR));
        }
        if (!hadDelayL && !hadDelayR && (hasDelayL || hasDelayR)) {
            _delayWriteIdx = 0;
        }

        _phaseDegL = _quantizePhase(p.phaseL);
        _phaseDegR = _quantizePhase(p.phaseR);
    }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        if (_subsonic.isEnabled()) {
            _subsonic.process(left, right, frames);
        } else {
            _scrubNonFinite(left, frames);
            _scrubNonFinite(right, frames);
        }

#if defined(HAVE_ESP_DSP)
        if (fabsf(_gainL - 1.0f) > 1e-4f) {
            dsps_mulc_f32(left,  left,  static_cast<int>(frames), _gainL, 1, 1);
        }
        if (fabsf(_gainR - 1.0f) > 1e-4f) {
            dsps_mulc_f32(right, right, static_cast<int>(frames), _gainR, 1, 1);
        }
#else
        if (fabsf(_gainL - 1.0f) > 1e-4f) {
            for (size_t i = 0; i < frames; ++i) {
                left[i] *= _gainL;
            }
        }
        if (fabsf(_gainR - 1.0f) > 1e-4f) {
            for (size_t i = 0; i < frames; ++i) {
                right[i] *= _gainR;
            }
        }
#endif

        const bool needDelayL = (_delaySamplesL > 0.01f);
        const bool needDelayR = (_delaySamplesR > 0.01f);

        if (needDelayL) {
            _processDelayChannel(left, frames, _delayBufL, _delayWriteIdx, _delaySamplesL);
        }
        if (needDelayR) {
            _processDelayChannel(right, frames, _delayBufR, _delayWriteIdx, _delaySamplesR);
        }
        if (needDelayL || needDelayR) {
            _delayWriteIdx = (_delayWriteIdx + frames) & (kDelayBufSize - 1);
        }

        _processPhaseChannel(left,  frames, _phaseDegL, _hStateL, needDelayL ? _delayBufL : nullptr);
        _processPhaseChannel(right, frames, _phaseDegR, _hStateR, needDelayR ? _delayBufR : nullptr);
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

    float          _hStateL[14]{};
    float          _hStateR[14]{};

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

    static inline void _scrubNonFinite(float* buf, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            if (!std::isfinite(buf[i])) buf[i] = 0.0f;
        }
    }

    static inline bool _sanitizeHilbertState(float* state, size_t count) {
        bool fault = false;
        for (size_t k = 0; k < count; ++k) {
            if (!std::isfinite(state[k])) {
                fault = true;
                break;
            }
        }
        if (fault) {
            memset(state, 0, count * sizeof(float));
            return true;
        }
        for (size_t k = 0; k < count; ++k) {
            if (fabsf(state[k]) < 1e-15f) {
                state[k] = 0.0f;
            }
        }
        return false;
    }

    static inline void _processDelayChannel(float* buf, size_t frames, float* delayBuf,
                                            size_t writeIdx, float delaySamples) {
        constexpr size_t kMask = kDelayBufSize - 1;
        const size_t delayInt = static_cast<size_t>(delaySamples);
        const float  frac     = delaySamples - static_cast<float>(delayInt);
        const float  invFrac  = 1.0f - frac;

        for (size_t i = 0; i < frames; ++i) {
            delayBuf[writeIdx] = buf[i];
            const size_t idxA = (writeIdx - delayInt) & kMask;
            const size_t idxB = (idxA - 1) & kMask;
            buf[i] = delayBuf[idxA] * invFrac + delayBuf[idxB] * frac;
            writeIdx = (writeIdx + 1) & kMask;
        }
    }

    static inline float _ap1(float x, float c, float& s) {
        const float y = c * x + s;
        s = x - c * y;
        return y;
    }

    static inline void _processPhaseChannel(float* buf, size_t frames, int16_t deg,
                                            float* state, float* delayBuf) {
        if (deg == 0) return;
        if (deg == 180 || deg == -180) {
#if defined(HAVE_ESP_DSP)
            dsps_mulc_f32(buf, buf, static_cast<int>(frames), -1.0f, 1, 1);
#else
            for (size_t i = 0; i < frames; ++i) {
                buf[i] = -buf[i];
            }
#endif
            return;
        }

        static constexpr float kSqrtHalf = 0.70710678f;
        float wI = 0.0f;
        float wQ = 0.0f;
        switch (deg) {
            case 90:   wI =  0.0f;       wQ = -1.0f;       break;
            case -90:  wI =  0.0f;       wQ =  1.0f;       break;
            case 45:   wI =  kSqrtHalf;  wQ = -kSqrtHalf;  break;
            case -45:  wI =  kSqrtHalf;  wQ =  kSqrtHalf;  break;
            case 135:  wI = -kSqrtHalf;  wQ = -kSqrtHalf;  break;
            case -135: wI = -kSqrtHalf;  wQ =  kSqrtHalf;  break;
            default:   return;
        }

        // 7+7 first-order all-pass Hilbert network tuned for 96 kHz (20 Hz..20 kHz, <= 1.1 deg error)
        static constexpr float kCoeffsI[7] = {
            -0.99972040f, -0.99822995f, -0.98883874f, -0.93133342f, -0.63232144f, 0.17593976f, 0.85340188f
        };
        static constexpr float kCoeffsQ[7] = {
            -0.99992052f, -0.99929639f, -0.99555072f, -0.97214268f, -0.83572099f, -0.27635952f, 0.56445904f
        };

        for (size_t i = 0; i < frames; ++i) {
            float xI = buf[i];
            float xQ = xI;
            for (int k = 0; k < 7; ++k) {
                xI = _ap1(xI, kCoeffsI[k], state[k]);
                xQ = _ap1(xQ, kCoeffsQ[k], state[7 + k]);
            }
            buf[i] = wI * xI + wQ * xQ;
        }

        if (_sanitizeHilbertState(state, 14)) {
            _scrubNonFinite(buf, frames);
            if (delayBuf) {
                _scrubNonFinite(delayBuf, kDelayBufSize);
            }
        }
    }
};
