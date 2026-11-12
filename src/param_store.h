#pragma once
#include "wireless_console.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>
#include <nvs.h>
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
    float threshold{-18.0f};
    float ratio{4.0f};
    float attackMs{5.0f};
    float releaseMs{150.0f};
    float makeupGain{0.0f};
    bool  enabled{false};
};

struct LimiterParams {
    float ceilingDb{-0.5f};
    float kneeDb{3.0f};
    bool  enabled{false};
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
    {
        _loadDefaults();
    }

    void begin() {
        _loadDefaults();

        auto nvsNamespaceExists = [](const char* ns) -> bool {
            nvs_handle_t h = 0;
            if (nvs_open(ns, NVS_READONLY, &h) == ESP_OK) {
                nvs_close(h);
                return true;
            }
            return false;
        };

        Preferences authPrefs;
        if (nvsNamespaceExists("dsp_auth") && authPrefs.begin("dsp_auth", true)) {
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
        if (nvsNamespaceExists("dsp_cfg") && prefs.begin("dsp_cfg", true)) {
            NvsBlobV2 blob{};
            bool loaded = false;
            if (prefs.getBytes("blob_v2", &blob, sizeof(blob)) == sizeof(blob) &&
                blob.magic == kNvsMagic && blob.version == kNvsVersion &&
                blob.crc32 == _calcBlobCrc(blob)) {
                xSemaphoreTake(_mutex, portMAX_DELAY);
                _eqEnabled = (blob.eqEnabled != 0);
                _eqCount   = (blob.eqCount >= 1 && blob.eqCount <= kEQBands) ? blob.eqCount : 5;
                _preampDb  = (std::isfinite(blob.preampDb) && blob.preampDb >= -18.0f && blob.preampDb <= 18.0f) ? blob.preampDb : 0.0f;
                memcpy(_eq, blob.eq, sizeof(_eq));
                for (int b = 0; b < kEQBands; ++b) _sanitizeEQBand(_eq[b]);
                _subsonic = blob.subsonic;
                _sanitizeSubsonic(_subsonic);
                _comp = blob.comp;
                _sanitizeCompander(_comp);
                _lim = blob.lim;
                _sanitizeLimiter(_lim);

                uint8_t savedInput = blob.inputMode;
                if (savedInput == INPUT_MODE_WIFI || savedInput == 0 || savedInput > 3) {
                    savedInput = INPUT_MODE_AUX;
                }
                _inputMode.store(static_cast<InputSourceMode>(savedInput), std::memory_order_relaxed);
                _activeSource.store(static_cast<ActiveInputSource>(savedInput), std::memory_order_relaxed);

                _bootCustomDefaults = (blob.bootCustomDefaults != 0);
                _bootEQ             = (blob.bootEQ != 0);
                _bootSubsonic       = (blob.bootSubsonic != 0);
                _bootComp           = (blob.bootComp != 0);
                _bootLim            = (blob.bootLim != 0);
                if (_bootCustomDefaults) {
                    _eqEnabled        = _bootEQ;
                    _subsonic.enabled = _bootSubsonic;
                    _comp.enabled     = _bootComp;
                    _lim.enabled      = _bootLim;
                }
                _sanitizeModuleOrder(blob.moduleOrder, _moduleOrder, sizeof(_moduleOrder));
                loaded = true;
                xSemaphoreGive(_mutex);
                Console.println("[NVS] Loaded versioned DSP config (v2) from flash.");
            } else if (prefs.isKey("init")) {
                xSemaphoreTake(_mutex, portMAX_DELAY);
                int eqCnt = prefs.getInt("eq_cnt", 5);
                if (eqCnt < 1 || eqCnt > kEQBands) eqCnt = 5;
                float preamp = prefs.getFloat("preamp", 0.0f);
                if (!std::isfinite(preamp) || preamp < -18.0f || preamp > 18.0f) preamp = 0.0f;

                if (prefs.getBytes("eq_bands", _eq, sizeof(_eq)) != sizeof(_eq)) {
                    _loadDefaults();
                } else {
                    _eqCount = eqCnt;
                    _preampDb = preamp;
                    for (int b = 0; b < kEQBands; ++b) _sanitizeEQBand(_eq[b]);

                    if (prefs.getBytes("subsonic", &_subsonic, sizeof(_subsonic)) != sizeof(_subsonic)) {
                        _subsonic = { 80.0f, 2, false };
                    } else {
                        _sanitizeSubsonic(_subsonic);
                    }

                    if (prefs.getBytes("comp", &_comp, sizeof(_comp)) != sizeof(_comp)) {
                        _comp = { -18.0f, 4.0f, 5.0f, 150.0f, 0.0f, false };
                    } else {
                        _sanitizeCompander(_comp);
                    }

                    if (prefs.getBytes("lim", &_lim, sizeof(_lim)) != sizeof(_lim)) {
                        _lim = { -0.5f, 3.0f, false };
                    } else {
                        _sanitizeLimiter(_lim);
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
                    _sanitizeModuleOrder(savedOrder.c_str(), _moduleOrder, sizeof(_moduleOrder));
                }
                loaded = true;
                xSemaphoreGive(_mutex);
                Console.println("[NVS] Migrated legacy DSP parameters from flash.");
            } else {
                Console.println("[NVS] No saved parameters found in flash. Defaults active (all effects OFF).");
            }
            prefs.end();

            if (loaded) {
                xSemaphoreTake(_mutex, portMAX_DELAY);
                _orderVersion.fetch_add(1, std::memory_order_release);
                _eqVersion.fetch_add(1, std::memory_order_release);
                _subsonicVersion.fetch_add(1, std::memory_order_release);
                _compVersion.fetch_add(1, std::memory_order_release);
                _limVersion.fetch_add(1, std::memory_order_release);
                _inputVersion.fetch_add(1, std::memory_order_release);
                xSemaphoreGive(_mutex);
            }
        }
    }

    void save() {
        NvsBlobV2 blob{};
        blob.magic   = kNvsMagic;
        blob.version = kNvsVersion;

        xSemaphoreTake(_mutex, portMAX_DELAY);
        const uint32_t saveSeq  = _editSeq.load(std::memory_order_acquire);
        blob.eqEnabled          = _eqEnabled ? 1 : 0;
        blob.eqCount            = static_cast<int32_t>(_eqCount);
        blob.preampDb           = _preampDb;
        memcpy(blob.eq, _eq, sizeof(blob.eq));
        blob.subsonic           = _subsonic;
        blob.comp               = _comp;
        blob.lim                = _lim;
        blob.inputMode          = static_cast<uint8_t>(_inputMode.load(std::memory_order_relaxed));
        blob.bootCustomDefaults = _bootCustomDefaults ? 1 : 0;
        blob.bootEQ             = _bootEQ ? 1 : 0;
        blob.bootSubsonic       = _bootSubsonic ? 1 : 0;
        blob.bootComp           = _bootComp ? 1 : 0;
        blob.bootLim            = _bootLim ? 1 : 0;
        memcpy(blob.moduleOrder, _moduleOrder, sizeof(blob.moduleOrder));
        xSemaphoreGive(_mutex);

        blob.crc32 = _calcBlobCrc(blob);

        Preferences prefs;
        if (!prefs.begin("dsp_cfg", false)) {
            return;
        }
        bool ok = (prefs.putBytes("blob_v2", &blob, sizeof(blob)) == sizeof(blob));
        prefs.end();

        if (ok) {
            xSemaphoreTake(_mutex, portMAX_DELAY);
            if (_editSeq.load(std::memory_order_acquire) == saveSeq) {
                _dirty.store(false, std::memory_order_release);
            }
            xSemaphoreGive(_mutex);
            Console.println("[NVS] Saved versioned DSP parameters to flash.");
        } else {
            Console.println("[NVS] ERROR: Failed to save DSP parameters to flash.");
        }
    }

    void markDirty() {
        _editSeq.fetch_add(1, std::memory_order_release);
        _lastDirtyTime.store(millis(), std::memory_order_relaxed);
        _dirty.store(true, std::memory_order_release);
    }

    bool isDirty() const {
        return _dirty.load(std::memory_order_acquire);
    }

    unsigned long getLastDirtyTime() const {
        return _lastDirtyTime.load(std::memory_order_relaxed);
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
        _markDirtyLocked();
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
        if (xSemaphoreTake(_mutex, 0) != pdTRUE) return false;
        strncpy(dest, _moduleOrder, maxLen - 1);
        dest[maxLen - 1] = '\0';
        localVersion = _orderVersion.load(std::memory_order_relaxed);
        xSemaphoreGive(_mutex);
        return true;
    }

    void setModuleOrder(const char* order) {
        if (!order || strlen(order) == 0) return;
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _sanitizeModuleOrder(order, _moduleOrder, sizeof(_moduleOrder));
        _orderVersion.fetch_add(1, std::memory_order_release);
        _markDirtyLocked();
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
        _markDirtyLocked();
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
        EQBandParams band{ freq, gain, q, true, type, order };
        _sanitizeEQBand(band);
        _eq[_eqCount] = band;
        _eqCount++;
        _eqVersion.fetch_add(1, std::memory_order_release);
        _markDirtyLocked();
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
        _markDirtyLocked();
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
            _sanitizeEQBand(sanitized);

            _eq[band] = sanitized;
            if (band >= _eqCount) _eqCount = band + 1;
            _eqVersion.fetch_add(1, std::memory_order_release);
            _markDirtyLocked();
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
        _markDirtyLocked();
        xSemaphoreGive(_mutex);
    }

    bool getEQStateIfChanged(EQBandParams* outBands, int& outCount, bool& outEnabled, float& outPreamp, uint32_t& lastVersion) const {
        if (_eqVersion.load(std::memory_order_acquire) == lastVersion) return false;
        if (xSemaphoreTake(_mutex, 0) != pdTRUE) return false;
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
        _markDirtyLocked();
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
        _sanitizeSubsonic(_subsonic);
        _subsonicVersion.fetch_add(1, std::memory_order_release);
        _markDirtyLocked();
        xSemaphoreGive(_mutex);
    }

    bool getSubsonicIfChanged(SubsonicParams& out, uint32_t& lastVersion) const {
        if (_subsonicVersion.load(std::memory_order_acquire) == lastVersion) return false;
        if (xSemaphoreTake(_mutex, 0) != pdTRUE) return false;
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
        _sanitizeCompander(_comp);
        _compVersion.fetch_add(1, std::memory_order_release);
        _markDirtyLocked();
        xSemaphoreGive(_mutex);
    }

    bool getCompanderIfChanged(CompanderParams& out, uint32_t& lastVersion) const {
        if (_compVersion.load(std::memory_order_acquire) == lastVersion) return false;
        if (xSemaphoreTake(_mutex, 0) != pdTRUE) return false;
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
        _sanitizeLimiter(_lim);
        _limVersion.fetch_add(1, std::memory_order_release);
        _markDirtyLocked();
        xSemaphoreGive(_mutex);
    }

    bool getLimiterIfChanged(LimiterParams& out, uint32_t& lastVersion) const {
        if (_limVersion.load(std::memory_order_acquire) == lastVersion) return false;
        if (xSemaphoreTake(_mutex, 0) != pdTRUE) return false;
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
            _markDirtyLocked();
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
    std::atomic<bool>              _dirty{false};
    std::atomic<unsigned long>     _lastDirtyTime{0};
    std::atomic<uint32_t>          _editSeq{0};
    std::atomic<uint32_t>          _eqVersion{1};
    std::atomic<uint32_t>          _subsonicVersion{1};
    std::atomic<uint32_t>          _compVersion{1};
    std::atomic<uint32_t>          _limVersion{1};
    std::atomic<uint32_t>          _inputVersion{1};
    std::atomic<uint32_t>          _wifiMetaVersion{1};
    char                           _moduleOrder[64]{"eq,subsonic,comp,lim"};
    std::atomic<uint32_t>          _orderVersion{1};

    static constexpr uint32_t kNvsMagic   = 0x44535032u;
    static constexpr uint16_t kNvsVersion = 2;

    struct NvsBlobV2 {
        uint32_t        magic;
        uint16_t        version;
        uint8_t         eqEnabled;
        uint8_t         inputMode;
        int32_t         eqCount;
        float           preampDb;
        EQBandParams    eq[kEQBands];
        SubsonicParams  subsonic;
        CompanderParams comp;
        LimiterParams   lim;
        uint8_t         bootCustomDefaults;
        uint8_t         bootEQ;
        uint8_t         bootSubsonic;
        uint8_t         bootComp;
        uint8_t         bootLim;
        char            moduleOrder[64];
        uint32_t        crc32;
    };

    static uint32_t _calcBlobCrc(const NvsBlobV2& blob) {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&blob);
        const size_t len = offsetof(NvsBlobV2, crc32);
        uint32_t crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < len; ++i) {
            crc ^= bytes[i];
            for (int b = 0; b < 8; ++b) {
                crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
            }
        }
        return ~crc;
    }

    void _markDirtyLocked() {
        _editSeq.fetch_add(1, std::memory_order_release);
        _lastDirtyTime.store(millis(), std::memory_order_relaxed);
        _dirty.store(true, std::memory_order_release);
    }

    static void _sanitizeEQBand(EQBandParams& b) {
        if (!std::isfinite(b.freq) || b.freq < 10.0f) b.freq = 10.0f;
        if (b.freq > 22000.0f) b.freq = 22000.0f;
        if (!std::isfinite(b.gain)) b.gain = 0.0f;
        if (b.gain < -24.0f) b.gain = -24.0f;
        if (b.gain >  24.0f) b.gain =  24.0f;
        if (!std::isfinite(b.q) || b.q < 0.05f) b.q = 0.05f;
        if (b.q > 10.0f) b.q = 10.0f;
        if (b.type > EQ_FILTER_FLAT_HIGH_PASS) b.type = EQ_FILTER_PEAK;
        if (b.order < 1 || b.order > 4) b.order = 2;
    }

    static void _sanitizeSubsonic(SubsonicParams& s) {
        if (!std::isfinite(s.freq) || s.freq < 20.0f) s.freq = 20.0f;
        if (s.freq > 100.0f) s.freq = 100.0f;
        if (s.order < 1 || s.order > 4) s.order = 2;
        if (!std::isfinite(s.balance)) s.balance = 0.0f;
        if (s.balance < -100.0f) s.balance = -100.0f;
        if (s.balance >  100.0f) s.balance =  100.0f;
        if (!std::isfinite(s.delayMsL)) s.delayMsL = 0.0f;
        if (s.delayMsL < -20.0f) s.delayMsL = -20.0f;
        if (s.delayMsL >  20.0f) s.delayMsL =  20.0f;
        if (!std::isfinite(s.delayMsR)) s.delayMsR = 0.0f;
        if (s.delayMsR < -20.0f) s.delayMsR = -20.0f;
        if (s.delayMsR >  20.0f) s.delayMsR =  20.0f;
        if (s.phaseL < -180) s.phaseL = -180;
        if (s.phaseL >  180) s.phaseL =  180;
        if (s.phaseR < -180) s.phaseR = -180;
        if (s.phaseR >  180) s.phaseR =  180;
    }

    static void _sanitizeCompander(CompanderParams& c) {
        if (!std::isfinite(c.threshold) || c.threshold < -60.0f) c.threshold = -60.0f;
        if (c.threshold > 0.0f) c.threshold = 0.0f;
        if (!std::isfinite(c.ratio) || c.ratio < 1.0f) c.ratio = 1.0f;
        if (c.ratio > 20.0f) c.ratio = 20.0f;
        if (!std::isfinite(c.attackMs) || c.attackMs < 0.1f) c.attackMs = 0.1f;
        if (c.attackMs > 500.0f) c.attackMs = 500.0f;
        if (!std::isfinite(c.releaseMs) || c.releaseMs < 10.0f) c.releaseMs = 10.0f;
        if (c.releaseMs > 2000.0f) c.releaseMs = 2000.0f;
        if (!std::isfinite(c.makeupGain)) c.makeupGain = 0.0f;
        if (c.makeupGain < -24.0f) c.makeupGain = -24.0f;
        if (c.makeupGain >  24.0f) c.makeupGain =  24.0f;
    }

    static void _sanitizeLimiter(LimiterParams& l) {
        if (!std::isfinite(l.ceilingDb) || l.ceilingDb < -30.0f) l.ceilingDb = -30.0f;
        if (l.ceilingDb > 0.0f) l.ceilingDb = 0.0f;
        if (!std::isfinite(l.kneeDb) || l.kneeDb < 0.1f) l.kneeDb = 0.1f;
        if (l.kneeDb > 12.0f) l.kneeDb = 12.0f;
    }

    static void _sanitizeModuleOrder(const char* src, char* dst, size_t dstLen) {
        if (!dst || dstLen == 0) return;
        if (!src || src[0] == '\0') {
            strncpy(dst, "eq,subsonic,comp,lim", dstLen - 1);
            dst[dstLen - 1] = '\0';
            return;
        }
        char tmp[64];
        strncpy(tmp, src, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';

        char out[64]{};
        size_t outPos = 0;
        bool seenEq = false, seenSub = false, seenComp = false, seenLim = false;

        char* savePtr = nullptr;
        char* tok = strtok_r(tmp, ",", &savePtr);
        while (tok) {
            while (*tok == ' ') tok++;
            if (*tok != '\0') {
                char* end = tok + strlen(tok) - 1;
                while (end > tok && *end == ' ') { *end = '\0'; end--; }
                const char* canonical = nullptr;
                if (strcmp(tok, "eq") == 0 && !seenEq) { seenEq = true; canonical = "eq"; }
                else if ((strcmp(tok, "subsonic") == 0 || strcmp(tok, "sub") == 0) && !seenSub) { seenSub = true; canonical = "subsonic"; }
                else if ((strcmp(tok, "comp") == 0 || strcmp(tok, "compressor") == 0) && !seenComp) { seenComp = true; canonical = "comp"; }
                else if ((strcmp(tok, "lim") == 0 || strcmp(tok, "limiter") == 0) && !seenLim) { seenLim = true; canonical = "lim"; }
                if (canonical) {
                    size_t clen = strlen(canonical);
                    if (outPos + (outPos > 0 ? 1 : 0) + clen < sizeof(out)) {
                        if (outPos > 0) out[outPos++] = ',';
                        memcpy(out + outPos, canonical, clen);
                        outPos += clen;
                    }
                }
            }
            tok = strtok_r(nullptr, ",", &savePtr);
        }
        out[outPos] = '\0';
        if (outPos == 0) {
            strncpy(dst, "eq,subsonic,comp,lim", dstLen - 1);
        } else {
            strncpy(dst, out, dstLen - 1);
        }
        dst[dstLen - 1] = '\0';
    }

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
