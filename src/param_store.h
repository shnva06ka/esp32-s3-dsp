// Settings shared between the audio task (core 1) and the web task (core 0). Mutex protected, saved to NVS.
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include "audio_config.h"

enum EQFilterType : uint8_t {
    EQ_FILTER_PEAK           = 0,
    EQ_FILTER_LOW_SHELF      = 1,
    EQ_FILTER_HIGH_SHELF     = 2,
    EQ_FILTER_LOW_PASS       = 3,
    EQ_FILTER_HIGH_PASS      = 4,
    EQ_FILTER_FLAT_LOW_PASS  = 5, // Legacy alias -> Low Pass (order=4)
    EQ_FILTER_FLAT_HIGH_PASS = 6  // Legacy alias -> High Pass (order=4)
};

struct EQBandParams {
    float   freq{1000.0f};       // Hz        (20 - 20000)
    float   gain{0.0f};          // dB        (-18 to +18)
    float   q{0.707f};           // Q factor  (0.1 - 10)
    bool    enabled{true};
    uint8_t type{EQ_FILTER_PEAK};
    uint8_t order{2};            // Slope order: 1 (6dB), 2 (12dB), 3 (18dB), 4 (24dB)

    EQBandParams() = default;
    EQBandParams(float f, float g, float q_, bool en = true, uint8_t t = EQ_FILTER_PEAK, uint8_t ord = 2)
        : freq(f), gain(g), q(q_), enabled(en), type(t), order(ord) {}
};

struct SubsonicParams {
    float   freq{80.0f};       // Hz (20 - 100)
    uint8_t order{2};          // 1, 2, 3, 4
    bool    enabled{false};    // Master module bypass

    float   balance{0.0f};     // -100 to +100 (-100 = L louder, +100 = R louder, 0 = center)
    float   delayMsL{0.0f};    // ms (-20.0 to +20.0)
    float   delayMsR{0.0f};    // ms (-20.0 to +20.0)
    int16_t phaseL{0};         // deg (-180 to +180 in 45-deg steps)
    int16_t phaseR{0};         // deg (-180 to +180 in 45-deg steps)

    SubsonicParams() = default;
    SubsonicParams(float f, uint8_t ord, bool en)
        : freq(f), order(ord), enabled(en) {}
};

struct CompanderParams {
    float threshold;  // dBFS  (-60 to 0)
    float ratio;      // N:1   (1 - 20)
    float attackMs;   // ms    (0.1 - 200)
    float releaseMs;  // ms    (10 - 2000)
    float makeupGain; // dB    (-12 to +24)
    bool  enabled;
};

struct LimiterParams {
    float ceilingDb;  // dBFS  (-6 to 0)
    float kneeDb;     // dB    (0 to 6) soft-knee width
    bool  enabled;
};

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

struct BluetoothMetadata {
    char title[128]{"No Track Playing"};
    char artist[128]{"Unknown Artist"};
    char album[128]{"Unknown Album"};
    bool connected{false};
    bool playing{false};
    uint32_t duration{0}; // seconds
    uint32_t pos{0};      // seconds
};

struct WiFiStreamMetadata {
    char serverName[64]{"None"};
    char serverIp[32]{""};
    char trackTitle[128]{"No Track Playing"};
    char trackArtist[128]{"Unknown Artist"};
    char trackAlbum[128]{"Unknown Album"};
    char formatInfo[64]{"PCM 44.1kHz 16-bit"};
    bool connected{false};
    bool streaming{false};
    bool paused{false};
    uint32_t sampleRate{44100};
    uint8_t bitDepth{16};
    uint32_t duration{0};
    uint32_t pos{0};
};

class ParamStore {
public:
    ParamStore()
        : _mutex(xSemaphoreCreateMutex())
    {}

    void begin() {
        _loadDefaults();

        Preferences prefs;
        if (prefs.begin("dsp_cfg", true)) {
            if (prefs.isKey("init")) {
                xSemaphoreTake(_mutex, portMAX_DELAY);
                _eqCount  = prefs.getInt("eq_cnt", 5);
                _preampDb = prefs.getFloat("preamp", 0.0f);
                prefs.getBytes("eq_bands", _eq, sizeof(_eq));
                prefs.getBytes("subsonic", &_subsonic, sizeof(_subsonic));
                prefs.getBytes("comp", &_comp, sizeof(_comp));
                prefs.getBytes("lim", &_lim, sizeof(_lim));
                _inputMode = static_cast<InputSourceMode>(prefs.getUChar("input", 1));
                _activeSource = static_cast<ActiveInputSource>(_inputMode);

                _bootCustomDefaults = prefs.getBool("boot_custom", false);
                _bootEQ             = prefs.getBool("boot_eq", false);
                _bootSubsonic       = prefs.getBool("boot_sub", false);
                _bootComp           = prefs.getBool("boot_comp", false);
                _bootLim            = prefs.getBool("boot_lim", false);

                if (_bootCustomDefaults) {
                    _eqEnabled        = _bootEQ;
                    _subsonic.enabled = _bootSubsonic;
                    _comp.enabled     = _bootComp;
                    _lim.enabled      = _bootLim;
                } else {
                    _eqEnabled = prefs.getBool("eq_en", false);
                }

                String savedOrder = prefs.getString("mod_order", "eq,subsonic,comp,lim");
                if (savedOrder.length() > 0) {
                    strncpy(_moduleOrder, savedOrder.c_str(), sizeof(_moduleOrder) - 1);
                    _moduleOrder[sizeof(_moduleOrder) - 1] = '\0';
                }
                _orderVersion++;

                _eqVersion++;
                _subsonicVersion++;
                _compVersion++;
                _limVersion++;
                _inputVersion++;
                xSemaphoreGive(_mutex);
                Serial.println("[NVS] Loaded all DSP parameters and states from flash.");
            } else {
                Serial.println("[NVS] No saved parameters found in flash. Defaults active (all effects OFF).");
            }
            prefs.end();
        }
    }

    void save() {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        Preferences prefs;
        if (!prefs.begin("dsp_cfg", false)) {
            xSemaphoreGive(_mutex);
            return;
        }
        prefs.putBool("init", true);
        prefs.putBool("eq_en", _eqEnabled);
        prefs.putInt("eq_cnt", _eqCount);
        prefs.putFloat("preamp", _preampDb);
        prefs.putBytes("eq_bands", _eq, sizeof(_eq));
        prefs.putBytes("subsonic", &_subsonic, sizeof(_subsonic));
        prefs.putBytes("comp", &_comp, sizeof(_comp));
        prefs.putBytes("lim", &_lim, sizeof(_lim));
        prefs.putUChar("input", static_cast<uint8_t>(_inputMode));
        prefs.putBool("boot_custom", _bootCustomDefaults);
        prefs.putBool("boot_eq", _bootEQ);
        prefs.putBool("boot_sub", _bootSubsonic);
        prefs.putBool("boot_comp", _bootComp);
        prefs.putBool("boot_lim", _bootLim);
        prefs.putString("mod_order", _moduleOrder);
        prefs.end();
        _dirty = false;
        xSemaphoreGive(_mutex);
        Serial.println("[NVS] Saved all DSP parameters and states to flash.");
    }

    void markDirty() {
        _dirty = true;
        _lastDirtyTime = millis();
    }

    bool isDirty() const {
        return _dirty;
    }

    unsigned long getLastDirtyTime() const {
        return _lastDirtyTime;
    }

    void getBootDefaults(bool& custom, bool& eq, bool& comp, bool& lim) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        custom = _bootCustomDefaults;
        eq     = _bootEQ;
        comp   = _bootComp;
        lim    = _bootLim;
        xSemaphoreGive(_mutex);
    }

    void getBootDefaults(bool& custom, bool& eq, bool& comp, bool& lim, bool& subsonic) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        custom   = _bootCustomDefaults;
        eq       = _bootEQ;
        comp     = _bootComp;
        lim      = _bootLim;
        subsonic = _bootSubsonic;
        xSemaphoreGive(_mutex);
    }

    void setBootDefaults(bool custom, bool eq, bool comp, bool lim, bool subsonic = false) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _bootCustomDefaults = custom;
        _bootEQ             = eq;
        _bootComp           = comp;
        _bootLim            = lim;
        _bootSubsonic       = subsonic;
        _dirty              = true;
        _lastDirtyTime      = millis();
        xSemaphoreGive(_mutex);
    }

    String getModuleOrder() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        String s = _moduleOrder;
        xSemaphoreGive(_mutex);
        return s;
    }

    void getModuleOrder(char* dest, size_t maxLen) const {
        if (!dest || maxLen == 0) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        strncpy(dest, _moduleOrder, maxLen - 1);
        dest[maxLen - 1] = '\0';
        xSemaphoreGive(_mutex);
    }

    bool getModuleOrderIfChanged(char* dest, size_t maxLen, uint32_t& localVersion) const {
        if (!dest || maxLen == 0) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (localVersion == _orderVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        strncpy(dest, _moduleOrder, maxLen - 1);
        dest[maxLen - 1] = '\0';
        localVersion = _orderVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    void setModuleOrder(const char* order) {
        if (!order || strlen(order) == 0) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        strncpy(_moduleOrder, order, sizeof(_moduleOrder) - 1);
        _moduleOrder[sizeof(_moduleOrder) - 1] = '\0';
        _orderVersion++;
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

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
        _dirty = true;
        _lastDirtyTime = millis();
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
        _dirty = true;
        _lastDirtyTime = millis();
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
        _dirty = true;
        _lastDirtyTime = millis();
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
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (band >= 0 && band < kEQBands) {
            _eq[band] = p;
            if (band >= _eqCount) _eqCount = band + 1;
            _eqVersion++;
            _dirty = true;
            _lastDirtyTime = millis();
        }
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
        if (db < -18.0f) db = -18.0f;
        if (db >  18.0f) db =  18.0f;
        _preampDb = db;
        _eqVersion++;
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getEQStateIfChanged(EQBandParams* outBands, int& outCount, bool& outEnabled, float& outPreamp, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_eqVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        outCount   = _eqCount;
        outEnabled = _eqEnabled;
        outPreamp  = _preampDb;
        for (int b = 0; b < _eqCount; ++b) {
            outBands[b] = _eq[b];
        }
        lastVersion = _eqVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

    void applyPreset(const char* name) {
        if (!name) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _eqEnabled = true;

        if (strcmp(name, "flat_10") == 0 || strcmp(name, "10_bands") == 0) {
            // ISO 10-band standard frequencies (31.5 Hz to 16 kHz)
            _eqCount = 10;
            _eq[0] = { 31.5f,   0.0f, 1.414f, true, EQ_FILTER_LOW_SHELF,  2 };
            _eq[1] = { 63.0f,   0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[2] = { 125.0f,  0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[3] = { 250.0f,  0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[4] = { 500.0f,  0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[5] = { 1000.0f, 0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[6] = { 2000.0f, 0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[7] = { 4000.0f, 0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[8] = { 8000.0f, 0.0f, 1.414f, true, EQ_FILTER_PEAK,       2 };
            _eq[9] = { 16000.0f,0.0f, 1.414f, true, EQ_FILTER_HIGH_SHELF, 2 };
        } else if (strcmp(name, "low_boost") == 0) {
            _eqCount = 5;
            _eq[0] = { 100.0f,   +6.0f, 0.707f, true, EQ_FILTER_LOW_SHELF,  2 };
            _eq[1] = { 500.0f,   +2.5f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[2] = { 1000.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[3] = { 4000.0f,  -1.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[4] = { 10000.0f, -2.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF, 2 };
        } else if (strcmp(name, "high_boost") == 0) {
            _eqCount = 5;
            _eq[0] = { 100.0f,   -2.0f, 0.707f, true, EQ_FILTER_LOW_SHELF,  2 };
            _eq[1] = { 500.0f,   -1.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[2] = { 1000.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[3] = { 4000.0f,  +3.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[4] = { 10000.0f, +6.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF, 2 };
        } else { // "flat_5", "flat", or default
            _eqCount = 5;
            _eq[0] = { 100.0f,   0.0f, 0.707f, true, EQ_FILTER_LOW_SHELF,  2 };
            _eq[1] = { 500.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[2] = { 1000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[3] = { 4000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[4] = { 10000.0f, 0.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF, 2 };
        }
        _eqVersion++;
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    SubsonicParams getSubsonic() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        auto p = _subsonic;
        xSemaphoreGive(_mutex);
        return p;
    }

    void setSubsonic(const SubsonicParams& p) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _subsonic = p;
        _subsonicVersion++;
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getSubsonicIfChanged(SubsonicParams& out, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_subsonicVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        out = _subsonic;
        lastVersion = _subsonicVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

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
        _dirty = true;
        _lastDirtyTime = millis();
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
        _dirty = true;
        _lastDirtyTime = millis();
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
        _dirty = true;
        _lastDirtyTime = millis();
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

    WiFiStreamMetadata getWiFiStreamMetadata() const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        WiFiStreamMetadata m = _wifiMeta;
        xSemaphoreGive(_mutex);
        return m;
    }

    void setWiFiStreamMetadata(const WiFiStreamMetadata& m) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _wifiMeta = m;
        _wifiMetaVersion++;
        xSemaphoreGive(_mutex);
    }

    bool getWiFiStreamMetadataIfChanged(WiFiStreamMetadata& out, uint32_t& lastVersion) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (_wifiMetaVersion == lastVersion) {
            xSemaphoreGive(_mutex);
            return false;
        }
        out = _wifiMeta;
        lastVersion = _wifiMetaVersion;
        xSemaphoreGive(_mutex);
        return true;
    }

private:
    mutable SemaphoreHandle_t _mutex;
    EQBandParams              _eq[kEQBands];
    int                       _eqCount{5};
    bool                      _eqEnabled{false};
    float                     _preampDb{0.0f};
    SubsonicParams            _subsonic;
    CompanderParams           _comp;
    LimiterParams             _lim;
    InputSourceMode           _inputMode{INPUT_MODE_AUX};
    ActiveInputSource         _activeSource{ACTIVE_SRC_AUX};
    BluetoothMetadata         _btMeta;
    WiFiStreamMetadata        _wifiMeta;
    bool                      _bootCustomDefaults{false};
    bool                      _bootEQ{false};
    bool                      _bootSubsonic{false};
    bool                      _bootComp{false};
    bool                      _bootLim{false};
    bool                      _dirty{false};
    unsigned long             _lastDirtyTime{0};
    uint32_t                  _eqVersion{1};
    uint32_t                  _subsonicVersion{1};
    uint32_t                  _compVersion{1};
    uint32_t                  _limVersion{1};
    uint32_t                  _inputVersion{1};
    uint32_t                  _btMetaVersion{1};
    uint32_t                  _wifiMetaVersion{1};
    char                      _moduleOrder[64]{"eq,subsonic,comp,lim"};
    uint32_t                  _orderVersion{1};

    void _loadDefaults() {
        // Default 5 initial bands: Low Shelf (100 Hz), 3 Peaking (500, 1000, 4000 Hz), High Shelf (10000 Hz)
        _eqCount   = 5;
        _eqEnabled = false;
        _preampDb  = 0.0f;
        _inputMode    = INPUT_MODE_AUX;
        _activeSource = ACTIVE_SRC_AUX;

        _eq[0] = { 100.0f,   0.0f, 0.707f, true, EQ_FILTER_LOW_SHELF,  2 };
        _eq[1] = { 500.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
        _eq[2] = { 1000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
        _eq[3] = { 4000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
        _eq[4] = { 10000.0f, 0.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF, 2 };

        constexpr float reserveFreqs[5] = { 250.0f, 2000.0f, 8000.0f, 16000.0f, 50.0f };
        for (int i = 5; i < kEQBands; ++i) {
            _eq[i] = { reserveFreqs[i - 5], 0.0f, 0.707f, true, EQ_FILTER_PEAK, 2 };
        }

        _subsonic = { 80.0f, 2, false };
        _comp     = { -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, false };
        _lim      = { -0.5f, 3.0f, false };

        _bootCustomDefaults = false;
        _bootEQ             = false;
        _bootSubsonic       = false;
        _bootComp           = false;
        _bootLim            = false;
        strncpy(_moduleOrder, "eq,subsonic,comp,lim", sizeof(_moduleOrder) - 1);
        _moduleOrder[sizeof(_moduleOrder) - 1] = '\0';
    }
};
