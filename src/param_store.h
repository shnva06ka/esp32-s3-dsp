#pragma once
#include "wireless_console.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <atomic>
#include <cstring>
#include <cmath>
#include "audio_config.h"

enum EQFilterType : uint8_t {
    EQ_FILTER_PEAK           = 0,
    EQ_FILTER_LOW_SHELF      = 1,
    EQ_FILTER_HIGH_SHELF     = 2,
    EQ_FILTER_LOW_PASS       = 3,
    EQ_FILTER_HIGH_PASS      = 4,
    EQ_FILTER_FLAT_LOW_PASS  = 5,
    EQ_FILTER_FLAT_HIGH_PASS = 6
};

struct EQBandParams {
    float   freq{1000.0f};
    float   gain{0.0f};
    float   q{0.707f};
    bool    enabled{true};
    uint8_t type{EQ_FILTER_PEAK};
    uint8_t order{2};

    EQBandParams() = default;
    EQBandParams(float f, float g, float q_, bool en = true, uint8_t t = EQ_FILTER_PEAK, uint8_t ord = 2)
        : freq(f), gain(g), q(q_), enabled(en), type(t), order(ord) {}
};

struct SubsonicParams {
    float   freq{80.0f};
    uint8_t order{2};
    bool    enabled{false};

    float   balance{0.0f};
    float   delayMsL{0.0f};
    float   delayMsR{0.0f};
    int16_t phaseL{0};
    int16_t phaseR{0};

    SubsonicParams() = default;
    SubsonicParams(float f, uint8_t ord, bool en)
        : freq(f), order(ord), enabled(en) {}
};

struct CompanderParams {
    float threshold;
    float ratio;
    float attackMs;
    float releaseMs;
    float makeupGain;
    bool  enabled;
};

struct LimiterParams {
    float ceilingDb;
    float kneeDb;
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

struct WiFiStreamMetadata {
    char serverName[64]{"None"};
    char serverIp[32]{""};
    char formatInfo[64]{"--"};
    bool connected{false};
    bool streaming{false};
    bool paused{false};
    uint32_t sampleRate{44100};
    uint8_t bitDepth{16};
    uint32_t pos{0};
};

class ParamStore {
public:
    ParamStore()
        : _mutex(xSemaphoreCreateMutex())
    {}

    void begin() {
        _loadDefaults();

        Preferences authPrefs;
        if (authPrefs.begin("dsp_auth", true)) {
            String u = authPrefs.getString("user", "admin");
            String p = authPrefs.getString("pass", "admin");
            xSemaphoreTake(_mutex, portMAX_DELAY);
            strncpy(_authUser, u.c_str(), sizeof(_authUser) - 1);
            _authUser[sizeof(_authUser) - 1] = '\0';
            strncpy(_authPass, p.c_str(), sizeof(_authPass) - 1);
            _authPass[sizeof(_authPass) - 1] = '\0';
            xSemaphoreGive(_mutex);
            authPrefs.end();
        }

        Preferences prefs;
        if (prefs.begin("dsp_cfg", true)) {
            if (prefs.isKey("init")) {
                xSemaphoreTake(_mutex, portMAX_DELAY);
                _eqCount  = prefs.getInt("eq_cnt", 5);
                if (_eqCount < 1 || _eqCount > kEQBands) _eqCount = 5;
                _preampDb = prefs.getFloat("preamp", 0.0f);
                if (!std::isfinite(_preampDb) || _preampDb < -18.0f || _preampDb > 18.0f) _preampDb = 0.0f;

                if (prefs.getBytes("eq_bands", _eq, sizeof(_eq)) != sizeof(_eq)) {
                    _loadDefaults();
                } else {
                    for (int b = 0; b < kEQBands; ++b) {
                        if (!std::isfinite(_eq[b].freq) || _eq[b].freq < 10.0f || _eq[b].freq > 22000.0f) _eq[b].freq = 1000.0f;
                        if (!std::isfinite(_eq[b].gain) || _eq[b].gain < -24.0f || _eq[b].gain > 24.0f) _eq[b].gain = 0.0f;
                        if (!std::isfinite(_eq[b].q) || _eq[b].q < 0.05f || _eq[b].q > 10.0f) _eq[b].q = 0.707f;
                        if (_eq[b].type > EQ_FILTER_FLAT_HIGH_PASS) _eq[b].type = EQ_FILTER_PEAK;
                        if (_eq[b].order < 1 || _eq[b].order > 4) _eq[b].order = 2;
                    }
                }

                if (prefs.getBytes("subsonic", &_subsonic, sizeof(_subsonic)) != sizeof(_subsonic)) {
                    _subsonic = { 80.0f, 2, false };
                } else {
                    if (!std::isfinite(_subsonic.freq) || _subsonic.freq < 10.0f || _subsonic.freq > 200.0f) _subsonic.freq = 80.0f;
                    if (_subsonic.order != 2 && _subsonic.order != 4) _subsonic.order = 2;
                    if (!std::isfinite(_subsonic.balance) || _subsonic.balance < -100.0f || _subsonic.balance > 100.0f) _subsonic.balance = 0.0f;
                    if (!std::isfinite(_subsonic.delayMsL) || _subsonic.delayMsL < -20.0f || _subsonic.delayMsL > 20.0f) _subsonic.delayMsL = 0.0f;
                    if (!std::isfinite(_subsonic.delayMsR) || _subsonic.delayMsR < -20.0f || _subsonic.delayMsR > 20.0f) _subsonic.delayMsR = 0.0f;
                }

                if (prefs.getBytes("comp", &_comp, sizeof(_comp)) != sizeof(_comp)) {
                    _comp = { -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, false };
                } else {
                    if (!std::isfinite(_comp.threshold) || _comp.threshold < -60.0f || _comp.threshold > 0.0f) _comp.threshold = -18.0f;
                    if (!std::isfinite(_comp.ratio) || _comp.ratio < 1.0f || _comp.ratio > 20.0f) _comp.ratio = 4.0f;
                    if (!std::isfinite(_comp.attackMs) || _comp.attackMs < 0.1f || _comp.attackMs > 500.0f) _comp.attackMs = 5.0f;
                    if (!std::isfinite(_comp.releaseMs) || _comp.releaseMs < 10.0f || _comp.releaseMs > 2000.0f) _comp.releaseMs = 150.0f;
                    if (!std::isfinite(_comp.makeupGain) || _comp.makeupGain < -24.0f || _comp.makeupGain > 24.0f) _comp.makeupGain = 0.0f;
                }

                if (prefs.getBytes("lim", &_lim, sizeof(_lim)) != sizeof(_lim)) {
                    _lim = { -0.5f, 3.0f, false };
                } else {
                    if (!std::isfinite(_lim.ceilingDb) || _lim.ceilingDb < -30.0f || _lim.ceilingDb > 0.0f) _lim.ceilingDb = -0.5f;
                    if (!std::isfinite(_lim.kneeDb) || _lim.kneeDb < 0.1f || _lim.kneeDb > 12.0f) _lim.kneeDb = 3.0f;
                }

                uint8_t savedInput = prefs.getUChar("input", 1);
                if (savedInput == INPUT_MODE_WIFI || savedInput == 0 || savedInput > 3) {
                    savedInput = INPUT_MODE_AUX;
                }
                _inputMode.store(static_cast<InputSourceMode>(savedInput), std::memory_order_relaxed);
                _activeSource.store(static_cast<ActiveInputSource>(savedInput), std::memory_order_relaxed);

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
                _orderVersion.fetch_add(1, std::memory_order_release);
                _eqVersion.fetch_add(1, std::memory_order_release);
                _subsonicVersion.fetch_add(1, std::memory_order_release);
                _compVersion.fetch_add(1, std::memory_order_release);
                _limVersion.fetch_add(1, std::memory_order_release);
                _inputVersion.fetch_add(1, std::memory_order_release);
                xSemaphoreGive(_mutex);
                Console.println("[NVS] Loaded all DSP parameters and states from flash.");
            } else {
                Console.println("[NVS] No saved parameters found in flash. Defaults active (all effects OFF).");
            }
            prefs.end();
        }
    }

    void save() {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        bool           eqEnabled = _eqEnabled;
        int            eqCount   = _eqCount;
        float          preampDb  = _preampDb;
        EQBandParams   eq[kEQBands];
        memcpy(eq, _eq, sizeof(eq));
        SubsonicParams  subsonic = _subsonic;
        CompanderParams comp     = _comp;
        LimiterParams   lim      = _lim;
        uint8_t         input    = static_cast<uint8_t>(_inputMode.load(std::memory_order_relaxed));
        bool            bCustom  = _bootCustomDefaults;
        bool            bEQ      = _bootEQ;
        bool            bSub     = _bootSubsonic;
        bool            bComp    = _bootComp;
        bool            bLim     = _bootLim;
        char            modOrder[64];
        memcpy(modOrder, _moduleOrder, sizeof(modOrder));
        xSemaphoreGive(_mutex);

        Preferences prefs;
        if (!prefs.begin("dsp_cfg", false)) {
            return;
        }
        prefs.putBool("init", true);
        prefs.putBool("eq_en", eqEnabled);
        prefs.putInt("eq_cnt", eqCount);
        prefs.putFloat("preamp", preampDb);
        prefs.putBytes("eq_bands", eq, sizeof(eq));
        prefs.putBytes("subsonic", &subsonic, sizeof(subsonic));
        prefs.putBytes("comp", &comp, sizeof(comp));
        prefs.putBytes("lim", &lim, sizeof(lim));
        prefs.putUChar("input", input);
        prefs.putBool("boot_custom", bCustom);
        prefs.putBool("boot_eq", bEQ);
        prefs.putBool("boot_sub", bSub);
        prefs.putBool("boot_comp", bComp);
        prefs.putBool("boot_lim", bLim);
        prefs.putString("mod_order", modOrder);
        prefs.end();

        xSemaphoreTake(_mutex, portMAX_DELAY);
        _dirty = false;
        xSemaphoreGive(_mutex);

        Console.println("[NVS] Saved all DSP parameters and states to flash.");
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
        if (_orderVersion.load(std::memory_order_acquire) == localVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        strncpy(dest, _moduleOrder, maxLen - 1);
        dest[maxLen - 1] = '\0';
        localVersion = _orderVersion.load(std::memory_order_relaxed);
        xSemaphoreGive(_mutex);
        return true;
    }

    void setModuleOrder(const char* order) {
        if (!order || strlen(order) == 0) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        strncpy(_moduleOrder, order, sizeof(_moduleOrder) - 1);
        _moduleOrder[sizeof(_moduleOrder) - 1] = '\0';
        _orderVersion.fetch_add(1, std::memory_order_release);
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
        _eqVersion.fetch_add(1, std::memory_order_release);
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
        _eqVersion.fetch_add(1, std::memory_order_release);
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
        _eqVersion.fetch_add(1, std::memory_order_release);
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
        return true;
    }

    EQBandParams getEQBand(int band) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        EQBandParams p{};
        if (band >= 0 && band < kEQBands) {
            p = _eq[band];
        }
        xSemaphoreGive(_mutex);
        return p;
    }

    void setEQBand(int band, const EQBandParams& p) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (band >= 0 && band < kEQBands) {
            EQBandParams sanitized = p;
            if (!std::isfinite(sanitized.freq) || sanitized.freq < 10.0f) sanitized.freq = 10.0f;
            if (sanitized.freq > 22000.0f) sanitized.freq = 22000.0f;
            if (!std::isfinite(sanitized.gain)) sanitized.gain = 0.0f;
            if (sanitized.gain < -24.0f) sanitized.gain = -24.0f;
            if (sanitized.gain > 24.0f) sanitized.gain = 24.0f;
            if (!std::isfinite(sanitized.q) || sanitized.q < 0.05f) sanitized.q = 0.05f;
            if (sanitized.q > 10.0f) sanitized.q = 10.0f;
            if (sanitized.type > EQ_FILTER_FLAT_HIGH_PASS) sanitized.type = EQ_FILTER_PEAK;
            if (sanitized.order < 1 || sanitized.order > 4) sanitized.order = 2;

            _eq[band] = sanitized;
            if (band >= _eqCount) _eqCount = band + 1;
            _eqVersion.fetch_add(1, std::memory_order_release);
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
        if (!std::isfinite(db)) db = 0.0f;
        if (db < -18.0f) db = -18.0f;
        if (db >  18.0f) db =  18.0f;
        _preampDb = db;
        _eqVersion.fetch_add(1, std::memory_order_release);
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getEQStateIfChanged(EQBandParams* outBands, int& outCount, bool& outEnabled, float& outPreamp, uint32_t& lastVersion) const {
        if (_eqVersion.load(std::memory_order_acquire) == lastVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        outCount   = _eqCount;
        outEnabled = _eqEnabled;
        outPreamp  = _preampDb;
        for (int b = 0; b < _eqCount; ++b) {
            outBands[b] = _eq[b];
        }
        lastVersion = _eqVersion.load(std::memory_order_relaxed);
        xSemaphoreGive(_mutex);
        return true;
    }

    void applyPreset(const char* name) {
        if (!name) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _eqEnabled = true;

        if (strcmp(name, "flat_10") == 0 || strcmp(name, "10_bands") == 0) {
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
        } else {
            _eqCount = 5;
            _eq[0] = { 100.0f,   0.0f, 0.707f, true, EQ_FILTER_LOW_SHELF,  2 };
            _eq[1] = { 500.0f,   0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[2] = { 1000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[3] = { 4000.0f,  0.0f, 0.707f, true, EQ_FILTER_PEAK,       2 };
            _eq[4] = { 10000.0f, 0.0f, 0.707f, true, EQ_FILTER_HIGH_SHELF, 2 };
        }
        _eqVersion.fetch_add(1, std::memory_order_release);
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
        if (!std::isfinite(_subsonic.freq) || _subsonic.freq < 10.0f) _subsonic.freq = 10.0f;
        if (_subsonic.freq > 200.0f) _subsonic.freq = 200.0f;
        if (_subsonic.order != 2 && _subsonic.order != 4) _subsonic.order = 2;
        if (!std::isfinite(_subsonic.balance)) _subsonic.balance = 0.0f;
        if (_subsonic.balance < -100.0f) _subsonic.balance = -100.0f;
        if (_subsonic.balance >  100.0f) _subsonic.balance =  100.0f;
        if (!std::isfinite(_subsonic.delayMsL)) _subsonic.delayMsL = 0.0f;
        if (_subsonic.delayMsL < -20.0f) _subsonic.delayMsL = -20.0f;
        if (_subsonic.delayMsL >  20.0f) _subsonic.delayMsL =  20.0f;
        if (!std::isfinite(_subsonic.delayMsR)) _subsonic.delayMsR = 0.0f;
        if (_subsonic.delayMsR < -20.0f) _subsonic.delayMsR = -20.0f;
        if (_subsonic.delayMsR >  20.0f) _subsonic.delayMsR =  20.0f;
        _subsonicVersion.fetch_add(1, std::memory_order_release);
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getSubsonicIfChanged(SubsonicParams& out, uint32_t& lastVersion) const {
        if (_subsonicVersion.load(std::memory_order_acquire) == lastVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        out = _subsonic;
        lastVersion = _subsonicVersion.load(std::memory_order_relaxed);
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
        if (!std::isfinite(_comp.threshold) || _comp.threshold < -60.0f) _comp.threshold = -60.0f;
        if (_comp.threshold > 0.0f) _comp.threshold = 0.0f;
        if (!std::isfinite(_comp.ratio) || _comp.ratio < 1.0f) _comp.ratio = 1.0f;
        if (_comp.ratio > 20.0f) _comp.ratio = 20.0f;
        if (!std::isfinite(_comp.attackMs) || _comp.attackMs < 0.1f) _comp.attackMs = 0.1f;
        if (_comp.attackMs > 500.0f) _comp.attackMs = 500.0f;
        if (!std::isfinite(_comp.releaseMs) || _comp.releaseMs < 10.0f) _comp.releaseMs = 10.0f;
        if (_comp.releaseMs > 2000.0f) _comp.releaseMs = 2000.0f;
        if (!std::isfinite(_comp.makeupGain)) _comp.makeupGain = 0.0f;
        if (_comp.makeupGain < -24.0f) _comp.makeupGain = -24.0f;
        if (_comp.makeupGain >  24.0f) _comp.makeupGain =  24.0f;
        _compVersion.fetch_add(1, std::memory_order_release);
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getCompanderIfChanged(CompanderParams& out, uint32_t& lastVersion) const {
        if (_compVersion.load(std::memory_order_acquire) == lastVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        out = _comp;
        lastVersion = _compVersion.load(std::memory_order_relaxed);
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
        if (!std::isfinite(_lim.ceilingDb) || _lim.ceilingDb < -30.0f) _lim.ceilingDb = -30.0f;
        if (_lim.ceilingDb > 0.0f) _lim.ceilingDb = 0.0f;
        if (!std::isfinite(_lim.kneeDb) || _lim.kneeDb < 0.1f) _lim.kneeDb = 0.1f;
        if (_lim.kneeDb > 12.0f) _lim.kneeDb = 12.0f;
        _limVersion.fetch_add(1, std::memory_order_release);
        _dirty = true;
        _lastDirtyTime = millis();
        xSemaphoreGive(_mutex);
    }

    bool getLimiterIfChanged(LimiterParams& out, uint32_t& lastVersion) const {
        if (_limVersion.load(std::memory_order_acquire) == lastVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        out = _lim;
        lastVersion = _limVersion.load(std::memory_order_relaxed);
        xSemaphoreGive(_mutex);
        return true;
    }

    InputSourceMode getInputMode() const {
        return _inputMode.load(std::memory_order_relaxed);
    }

    void setInputMode(InputSourceMode m, bool persist = true) {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _inputMode.store(m, std::memory_order_relaxed);
        _inputVersion.fetch_add(1, std::memory_order_release);
        if (persist) {
            _dirty = true;
            _lastDirtyTime = millis();
        }
        xSemaphoreGive(_mutex);
    }

    ActiveInputSource getActiveSource() const {
        return _activeSource.load(std::memory_order_relaxed);
    }

    void setActiveSource(ActiveInputSource s) {
        if (_activeSource.load(std::memory_order_relaxed) != s) {
            _activeSource.store(s, std::memory_order_relaxed);
            _inputVersion.fetch_add(1, std::memory_order_release);
        }
    }

    bool getInputStateIfChanged(InputSourceMode& mode, ActiveInputSource& active, uint32_t& lastVersion) const {
        uint32_t v = _inputVersion.load(std::memory_order_acquire);
        if (v == lastVersion) return false;
        mode   = _inputMode.load(std::memory_order_relaxed);
        active = _activeSource.load(std::memory_order_relaxed);
        lastVersion = v;
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
        _wifiMetaVersion.fetch_add(1, std::memory_order_release);
        xSemaphoreGive(_mutex);
    }

    bool getWiFiStreamMetadataIfChanged(WiFiStreamMetadata& out, uint32_t& lastVersion) const {
        if (_wifiMetaVersion.load(std::memory_order_acquire) == lastVersion) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        out = _wifiMeta;
        lastVersion = _wifiMetaVersion.load(std::memory_order_relaxed);
        xSemaphoreGive(_mutex);
        return true;
    }

    void getAuth(char* outUser, size_t userLen, char* outPass, size_t passLen) const {
        xSemaphoreTake(_mutex, portMAX_DELAY);
        if (outUser && userLen > 0) {
            strncpy(outUser, _authUser, userLen - 1);
            outUser[userLen - 1] = '\0';
        }
        if (outPass && passLen > 0) {
            strncpy(outPass, _authPass, passLen - 1);
            outPass[passLen - 1] = '\0';
        }
        xSemaphoreGive(_mutex);
    }

    const char* getAuthUser() const { return _authUser; }
    const char* getAuthPass() const { return _authPass; }

    bool checkAuth(const char* user, const char* pass) const {
        if (!user || !pass) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        bool ok = (strcmp(user, _authUser) == 0 && strcmp(pass, _authPass) == 0);
        xSemaphoreGive(_mutex);
        return ok;
    }

    bool setAuth(const char* user, const char* pass) {
        if (!user || strlen(user) < 1 || strlen(user) >= sizeof(_authUser)) return false;
        if (!pass || strlen(pass) < 4 || strlen(pass) >= sizeof(_authPass)) return false;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        strncpy(_authUser, user, sizeof(_authUser) - 1);
        _authUser[sizeof(_authUser) - 1] = '\0';
        strncpy(_authPass, pass, sizeof(_authPass) - 1);
        _authPass[sizeof(_authPass) - 1] = '\0';
        xSemaphoreGive(_mutex);

        Preferences authPrefs;
        if (authPrefs.begin("dsp_auth", false)) {
            authPrefs.putString("user", user);
            authPrefs.putString("pass", pass);
            authPrefs.end();
            return true;
        }
        return false;
    }

private:
    mutable SemaphoreHandle_t      _mutex;
    char                           _authUser[32]{"admin"};
    char                           _authPass[64]{"admin"};
    EQBandParams                   _eq[kEQBands];
    int                            _eqCount{5};
    bool                           _eqEnabled{false};
    float                          _preampDb{0.0f};
    SubsonicParams                 _subsonic;
    CompanderParams                _comp;
    LimiterParams                  _lim;
    std::atomic<InputSourceMode>   _inputMode{INPUT_MODE_AUX};
    std::atomic<ActiveInputSource> _activeSource{ACTIVE_SRC_AUX};
    WiFiStreamMetadata             _wifiMeta;
    bool                           _bootCustomDefaults{false};
    bool                           _bootEQ{false};
    bool                           _bootSubsonic{false};
    bool                           _bootComp{false};
    bool                           _bootLim{false};
    bool                           _dirty{false};
    unsigned long                  _lastDirtyTime{0};
    std::atomic<uint32_t>          _eqVersion{1};
    std::atomic<uint32_t>          _subsonicVersion{1};
    std::atomic<uint32_t>          _compVersion{1};
    std::atomic<uint32_t>          _limVersion{1};
    std::atomic<uint32_t>          _inputVersion{1};
    std::atomic<uint32_t>          _wifiMetaVersion{1};
    char                           _moduleOrder[64]{"eq,subsonic,comp,lim"};
    std::atomic<uint32_t>          _orderVersion{1};

    void _loadDefaults() {
        _eqCount   = 5;
        _eqEnabled = false;
        _preampDb  = 0.0f;
        _inputMode.store(INPUT_MODE_AUX, std::memory_order_relaxed);
        _activeSource.store(ACTIVE_SRC_AUX, std::memory_order_relaxed);

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
