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
        if (!std::isfinite(sumL) || sumL < 0.0f || !std::isfinite(sumR) || sumR < 0.0f) {
            sumL = 0.0f;
            sumR = 0.0f;
            for (size_t i = 0; i < frames; ++i) {
                if (!std::isfinite(left[i]))  left[i]  = 0.0f;
                if (!std::isfinite(right[i])) right[i] = 0.0f;
                sumL += left[i]  * left[i];
                sumR += right[i] * right[i];
            }
        }

        const float rmsL = sqrtf(sumL / static_cast<float>(frames));
        const float rmsR = sqrtf(sumR / static_cast<float>(frames));
        float rms  = (rmsL > rmsR) ? rmsL : rmsR;
        if (!std::isfinite(rms)) rms = 0.0f;

        if (!std::isfinite(_envRMS)) _envRMS = 0.0f;
        if (!std::isfinite(_currentGain) || _currentGain < 0.0f) _currentGain = 1.0f;
        float attackCoeff  = _attackCoeff;
        float releaseCoeff = _releaseCoeff;
        if (frames != kBufferFrames) {
            const float blockRate = static_cast<float>(kSampleRate) / static_cast<float>(frames);
            attackCoeff  = expf(-2.2f / (blockRate * (_attackMs * 0.001f)));
            releaseCoeff = expf(-2.2f / (blockRate * (_releaseMs * 0.001f)));
        }
        const float coeff = (rms > _envRMS) ? attackCoeff : releaseCoeff;
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
    float _attackMs{5.0f};
    float _releaseMs{150.0f};
    float _attackCoeff{0.f};
    float _releaseCoeff{0.f};
    float _envRMS{0.f};
    float _currentGain{1.0f};

    void _update(const CompanderParams& p) {
        _enabled      = p.enabled;
        const float rawThresh = std::isfinite(p.threshold)  ? p.threshold  : -18.0f;
        const float rawRatio  = std::isfinite(p.ratio)      ? p.ratio      : 4.0f;
        const float rawMakeup = std::isfinite(p.makeupGain) ? p.makeupGain : 0.0f;
        const float rawAtt    = std::isfinite(p.attackMs)   ? p.attackMs   : 5.0f;
        const float rawRel    = std::isfinite(p.releaseMs)  ? p.releaseMs  : 150.0f;

        _thresholdDb  = (rawThresh < -60.0f) ? -60.0f : ((rawThresh > 0.0f) ? 0.0f : rawThresh);
        _ratio        = (rawRatio < 1.0f) ? 1.0f : ((rawRatio > 20.0f) ? 20.0f : rawRatio);
        _makeupGainDb = (rawMakeup < -24.0f) ? -24.0f : ((rawMakeup > 24.0f) ? 24.0f : rawMakeup);

        _attackMs  = (rawAtt < 0.1f) ? 0.1f : ((rawAtt > 500.0f) ? 500.0f : rawAtt);
        _releaseMs = (rawRel < 10.0f) ? 10.0f : ((rawRel > 2000.0f) ? 2000.0f : rawRel);

        const float F_block = static_cast<float>(kSampleRate) / static_cast<float>(kBufferFrames);
        auto tc = [&](float ms) -> float {
            float sec = ms / 1000.0f;
            return expf(-2.2f / (F_block * sec));
        };
        _attackCoeff  = tc(_attackMs);
        _releaseCoeff = tc(_releaseMs);
    }
};
