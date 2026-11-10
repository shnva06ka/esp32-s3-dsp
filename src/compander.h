#pragma once

#include <cmath>
#if __has_include(<esp_dsp.h>)
#include <esp_dsp.h>
#define HAVE_ESP_DSP 1
#endif
#include "audio_config.h"
#include "param_store.h"

class Compander {
public:
    Compander() { _update({ -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true }); }

    void reset() {
        _envRMS = 0.0f;
        _currentGain = 1.0f;
    }

    void setParams(const CompanderParams& p) { _update(p); }

    void process(float* left, float* right, size_t frames) {
        if (!_enabled || frames == 0) return;

        float sumL = 0.0f, sumR = 0.0f;
#if defined(HAVE_ESP_DSP)
        dsps_dotprod_f32(left,  left,  &sumL, static_cast<int>(frames));
        dsps_dotprod_f32(right, right, &sumR, static_cast<int>(frames));
#else
        for (size_t i = 0; i < frames; ++i) {
            sumL += left[i]  * left[i];
            sumR += right[i] * right[i];
        }
#endif
        if (!std::isfinite(sumL) || sumL < 0.0f) sumL = 0.0f;
        if (!std::isfinite(sumR) || sumR < 0.0f) sumR = 0.0f;

        const float rmsL = sqrtf(sumL / static_cast<float>(frames));
        const float rmsR = sqrtf(sumR / static_cast<float>(frames));
        float rms  = (rmsL > rmsR) ? rmsL : rmsR;
        if (!std::isfinite(rms)) rms = 0.0f;

        if (!std::isfinite(_envRMS)) _envRMS = 0.0f;
        const float coeff = (rms > _envRMS) ? _attackCoeff : _releaseCoeff;
        _envRMS = _envRMS * coeff + rms * (1.0f - coeff);
        if (fabsf(_envRMS) < 1e-15f) _envRMS = 0.0f;

        const float levelDb = 20.0f * log10f(_envRMS + 1e-9f);
        float gainDb = 0.0f;
        if (levelDb > _thresholdDb) {
            gainDb = (_thresholdDb - levelDb) * (1.0f - 1.0f / _ratio);
        }
        gainDb += _makeupGainDb;

        float targetGain = powf(10.0f, gainDb / 20.0f);
        if (!std::isfinite(targetGain) || targetGain < 0.0f) targetGain = 1.0f;
        if (targetGain > 16.0f) targetGain = 16.0f;

        if (fabsf(targetGain - _currentGain) < 1e-5f) {
            _currentGain = targetGain;
            if (fabsf(_currentGain - 1.0f) > 1e-4f) {
#if defined(HAVE_ESP_DSP)
                dsps_mulc_f32(left,  left,  static_cast<int>(frames), _currentGain, 1, 1);
                dsps_mulc_f32(right, right, static_cast<int>(frames), _currentGain, 1, 1);
#else
                for (size_t i = 0; i < frames; ++i) {
                    left[i]  *= _currentGain;
                    right[i] *= _currentGain;
                }
#endif
            }
        } else {
            const float gainStep = (targetGain - _currentGain) / static_cast<float>(frames);
            for (size_t i = 0; i < frames; ++i) {
                _currentGain += gainStep;
                left[i]  *= _currentGain;
                right[i] *= _currentGain;
            }
        }
    }

private:
    bool  _enabled{true};
    float _thresholdDb{-18.f};
    float _ratio{4.f};
    float _makeupGainDb{0.f};
    float _attackCoeff{0.f};
    float _releaseCoeff{0.f};
    float _envRMS{0.f};
    float _currentGain{1.0f};

    void _update(const CompanderParams& p) {
        _enabled      = p.enabled;
        _thresholdDb  = (p.threshold < -60.0f) ? -60.0f : ((p.threshold > 0.0f) ? 0.0f : p.threshold);
        _ratio        = (p.ratio < 1.0f) ? 1.0f : ((p.ratio > 20.0f) ? 20.0f : p.ratio);
        _makeupGainDb = (p.makeupGain < -24.0f) ? -24.0f : ((p.makeupGain > 24.0f) ? 24.0f : p.makeupGain);

        float safeAttack  = (p.attackMs < 0.1f) ? 0.1f : ((p.attackMs > 500.0f) ? 500.0f : p.attackMs);
        float safeRelease = (p.releaseMs < 10.0f) ? 10.0f : ((p.releaseMs > 2000.0f) ? 2000.0f : p.releaseMs);

        const float F_block = static_cast<float>(kSampleRate) / static_cast<float>(kBufferFrames);
        auto tc = [&](float ms) -> float {
            float sec = ms / 1000.0f;
            return expf(-2.2f / (F_block * sec));
        };
        _attackCoeff  = tc(safeAttack);
        _releaseCoeff = tc(safeRelease);
    }
};
