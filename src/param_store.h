/**
 * @file param_store.h
 * @brief Thread-safe shared parameter store (EQ + compander + limiter).
 *
 * Reads/writes use a FreeRTOS mutex so the audio task (Core 1) and
 * the web task (Core 0) can exchange parameters without data races.
 * NVS persistence is planned but stubbed — call g_params.save() to
 * commit, g_params.begin() to load.
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include "audio_config.h"

// ── EQ Filter Types ──────────────────────────────────────────
enum EQFilterType : uint8_t {
    EQ_FILTER_PEAK           = 0,
    EQ_FILTER_LOW_SHELF      = 1,
    EQ_FILTER_HIGH_SHELF     = 2,
    EQ_FILTER_LOW_PASS       = 3,
    EQ_FILTER_HIGH_PASS      = 4,
    EQ_FILTER_FLAT_LOW_PASS  = 5, // Legacy alias -> Low Pass (order=4)
    EQ_FILTER_FLAT_HIGH_PASS = 6  // Legacy alias -> High Pass (order=4)
};

// ── Per-band EQ parameters ───────────────────────────────────
struct EQBandParams {
    float   freq{1000.0f};       // Hz        (20 – 20 000)
    float   gain{0.0f};          // dB        (-18 … +18)
    float   q{0.707f};           // Q factor  (0.1 – 10)
    bool    enabled{true};
    uint8_t type{EQ_FILTER_PEAK};
    uint8_t order{2};            // Slope order: 1 (6dB), 2 (12dB), 3 (18dB), 4 (24dB)

    EQBandParams() = default;
    EQBandParams(float f, float g, float q_, bool en = true, uint8_t t = EQ_FILTER_PEAK, uint8_t ord = 2)
        : freq(f), gain(g), q(q_), enabled(en), type(t), order(ord) {}
};

// ── Compander parameters ─────────────────────────────────────
struct CompanderParams {
    float threshold;  // dBFS  (-60 … 0)
    float ratio;      // N:1   (1 – 20)
    float attackMs;   // ms    (0.1 – 200)
    float releaseMs;  // ms    (10  – 2000)
    float makeupGain; // dB    (-12 … +24)
    bool  enabled;
};

// ── Soft-limiter parameters ──────────────────────────────────
struct LimiterParams {
    float ceilingDb;  // dBFS  (-6 … 0)
    float kneeDb;     // dB    (0 … 6) soft-knee width
    bool  enabled;
};

// ── Input Source Modes & Active Sources ──────────────────────
enum InputSourceMode : uint8_t {
    INPUT_MODE_AUX  = 1,
    INPUT_MODE_BT   = 2,
    INPUT_MODE_WIFI = 3
};

enum ActiveInputSource : uint8_t {
    ACTIVE_SRC_AUX  = 1,
    ACTIVE_SRC_BT   = 2,
    ACTIVE_SRC_WIFI = 3
};

// ── Bluetooth Metadata Structure ─────────────────────────────
struct BluetoothMetadata {
    char title[128]{"No Track Playing"};
    char artist[128]{"Unknown Artist"};
    char album[128]{"Unknown Album"};
    bool connected{false};
    bool playing{false};
    uint32_t duration{0}; // seconds
    uint32_t pos{0};      // seconds
};

// ─────────────────────────────────────────────────────────────
class ParamStore {
public:
    ParamStore()
        : _mutex(xSemaphoreCreateMutex())
    {}

    // ── Lifecycle ───────────────────────────────────────────
    void begin() {
        // Load defaults; replace with NVS reads for persistence
        _loadDefaults();
    }

    void save() {
        // TODO: persist to NVS via Preferences
    }

    // ── EQ ──────────────────────────────────────────────────
    bool getEQEnabled() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        bool en = _eqEnabled;
        xSemaphoreGive(_mutex);
        return en;
    }

    void setEQEnabled(bool en) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _eqEnabled = en;
        _eqVersion++;
        xSemaphoreGive(_mutex);
    }

    int getEQBandCount() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        int c = _eqCount;
        xSemaphoreGive(_mutex);
        return c;
    }

    bool addEQBand(float freq, float gain, float q, uint8_t type = EQ_FILTER_PEAK, uint8_t order = 2) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_eqCount >= kEQBands) {
            xSemaphoreGive(_mutex);
            return false;
        }
        _eq[_eqCount] = { freq, gain, q, true, type, order };
        _eqCount++;
        _eqVersion++;
        xSemaphoreGive(_mutex);
        return true;
    }

    bool removeEQBand(int index) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (index < 0 || index >= _eqCount || _eqCount <= 1) {
            xSemaphoreGive(_mutex);
            return false;
        }
        for (int i = index; i < _eqCount - 1; ++i) {
            _eq[i] = _eq[i + 1];
        }
        _eqCount--;
        _eqVersion++;
        xSemaphoreGive(_mutex);
        return true;
    }

    EQBandParams getEQBand(int band) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto p = _eq[band];
        xSemaphoreGive(_mutex);
        return p;
    }

    void setEQBand(int band, const EQBandParams& p) {
        if (band < 0 || band >= kEQBands) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _eq[band] = p;
        if (band >= _eqCount) _eqCount = band + 1;
        _eqVersion++;
        xSemaphoreGive(_mutex);
    }

    float getPreamp() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        float p = _preampDb;
        xSemaphoreGive(_mutex);
        return p;
    }

    void setPreamp(float db) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _preampDb = (db < -18.0f) ? -18.0f : ((db > 18.0f) ? 18.0f : db);
        _eqVersion++;
        xSemaphoreGive(_mutex);
    }

    bool getEQStateIfChanged(EQBandParams* out, int& count, bool& enabled, float& preampDb, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_eqVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        count    = _eqCount;
        enabled  = _eqEnabled;
        preampDb = _preampDb;
        for (int i = 0; i < _eqCount; ++i) out[i] = _eq[i];
        lastVersion = _eqVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    bool getEQStateIfChanged(EQBandParams* out, int& count, bool& enabled, uint32_t& lastVersion) const {
        float dummyPreamp;
        return getEQStateIfChanged(out, count, enabled, dummyPreamp, lastVersion);
    }

    bool getEQBandsIfChanged(EQBandParams* out, uint32_t& lastVersion) const {
        int dummyCount;
        bool dummyEn;
        float dummyPreamp;
        return getEQStateIfChanged(out, dummyCount, dummyEn, dummyPreamp, lastVersion);
    }

    void applyPreset(const char* name) {
        if (!name) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _eqCount = 5;
        _eqEnabled = true;

        if (strcmp(name, "low_boost") == 0) {
            _eq[0] = { 100.0f,   +6.0f, 0.707f, true, EQ_FILTER_LOW_SHELF };
            _eq[1] = { 500.0f,   +2.5f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[2] = { 1000.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[3] = { 4000.0f,  -1.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[4] = { 10000.0f, -2.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF };
        } else if (strcmp(name, "high_boost") == 0) {
            _eq[0] = { 100.0f,   -2.0f, 0.707f, true, EQ_FILTER_LOW_SHELF };
            _eq[1] = { 500.0f,   -1.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[2] = { 1000.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[3] = { 4000.0f,  +3.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[4] = { 10000.0f, +6.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF };
        } else { // "flat" or default
            _eq[0] = { 100.0f,   0.0f, 0.707f, true, EQ_FILTER_LOW_SHELF };
            _eq[1] = { 500.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[2] = { 1000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[3] = { 4000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK };
            _eq[4] = { 10000.0f, 0.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF };
        }
        _eqVersion++;
        xSemaphoreGive(_mutex);
    }

    // ── Compander ───────────────────────────────────────────
    CompanderParams getCompander() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto p = _comp;
        xSemaphoreGive(_mutex);
        return p;
    }

    void setCompander(const CompanderParams& p) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _comp = p;
        _compVersion++;
        xSemaphoreGive(_mutex);
    }

    bool getCompanderIfChanged(CompanderParams& out, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_compVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        out = _comp;
        lastVersion = _compVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    // ── Limiter ─────────────────────────────────────────────
    LimiterParams getLimiter() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto p = _lim;
        xSemaphoreGive(_mutex);
        return p;
    }

    void setLimiter(const LimiterParams& p) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _lim = p;
        _limVersion++;
        xSemaphoreGive(_mutex);
    }

    bool getLimiterIfChanged(LimiterParams& out, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_limVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        out = _lim;
        lastVersion = _limVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    // ── Input Source ────────────────────────────────────────
    InputSourceMode getInputMode() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto m = _inputMode;
        xSemaphoreGive(_mutex);
        return m;
    }

    void setInputMode(InputSourceMode m) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _inputMode = m;
        _inputVersion++;
        xSemaphoreGive(_mutex);
    }

    ActiveInputSource getActiveSource() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto s = _activeSource;
        xSemaphoreGive(_mutex);
        return s;
    }

    void setActiveSource(ActiveInputSource s) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_activeSource != s) {
            _activeSource = s;
            _inputVersion++;
        }
        xSemaphoreGive(_mutex);
    }

    bool getInputStateIfChanged(InputSourceMode& mode, ActiveInputSource& active, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_inputVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        mode = _inputMode;
        active = _activeSource;
        lastVersion = _inputVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    // ── Bluetooth Metadata ──────────────────────────────────
    BluetoothMetadata getBluetoothMetadata() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        BluetoothMetadata m = _btMeta;
        xSemaphoreGive(_mutex);
        return m;
    }

    void setBluetoothMetadata(const BluetoothMetadata& m) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _btMeta = m;
        _btMetaVersion++;
        xSemaphoreGive(_mutex);
    }

    bool getBluetoothMetadataIfChanged(BluetoothMetadata& out, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_btMetaVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        out = _btMeta;
        lastVersion = _btMetaVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

private:
    mutable SemaphoreHandle_t _mutex;
    EQBandParams              _eq[kEQBands];
    int                       _eqCount{5};       // Starts with 5 bands by default
    bool                      _eqEnabled{true};  // Global EQ module bypass
    float                     _preampDb{0.0f};   // EQ preamp gain (-18 to +18 dB)
    CompanderParams           _comp;
    LimiterParams             _lim;
    InputSourceMode           _inputMode{INPUT_MODE_AUX};
    ActiveInputSource         _activeSource{ACTIVE_SRC_AUX};
    BluetoothMetadata         _btMeta;
    uint32_t                  _eqVersion{1};
    uint32_t                  _compVersion{1};
    uint32_t                  _limVersion{1};
    uint32_t                  _inputVersion{1};
    uint32_t                  _btMetaVersion{1};

    void _loadDefaults() {
        // Default 5 initial bands: Low Shelf (100 Hz), 3 Peaking (500, 1000, 4000 Hz), High Shelf (10000 Hz)
        _eqCount   = 5;
        _eqEnabled = true;
        _preampDb  = 0.0f;
        _inputMode    = INPUT_MODE_AUX;
        _activeSource = ACTIVE_SRC_AUX;

        _eq[0] = { 100.0f,   0.0f, 0.707f, true, EQ_FILTER_LOW_SHELF };
        _eq[1] = { 500.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK };
        _eq[2] = { 1000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK };
        _eq[3] = { 4000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK };
        _eq[4] = { 10000.0f, 0.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF };

        // Remaining backup bands
        constexpr float reserveFreqs[5] = { 250.0f, 2000.0f, 8000.0f, 16000.0f, 50.0f };
        for (int i = 5; i < kEQBands; ++i) {
            _eq[i] = { reserveFreqs[i - 5], 0.0f, 0.707f, true, EQ_FILTER_PEAK };
        }

        _comp = { -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, true };
        _lim  = { -0.5f, 3.0f, true };
    }
};

