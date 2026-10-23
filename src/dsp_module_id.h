#pragma once

#include <cstdint>
#include <cstring>

enum class DspModuleId : uint8_t {
    Eq = 0,
    Subsonic,
    Compander,
    Limiter,
    Unknown
};

static inline DspModuleId parseDspModuleId(const char* token) {
    if (!token || *token == '\0') return DspModuleId::Unknown;
    if (strcmp(token, "eq") == 0) return DspModuleId::Eq;
    if (strcmp(token, "subsonic") == 0 || strcmp(token, "sub") == 0) return DspModuleId::Subsonic;
    if (strcmp(token, "comp") == 0 || strcmp(token, "compressor") == 0) return DspModuleId::Compander;
    if (strcmp(token, "lim") == 0 || strcmp(token, "limiter") == 0) return DspModuleId::Limiter;
    return DspModuleId::Unknown;
}

static inline const char* toDspModuleToken(DspModuleId id) {
    switch (id) {
        case DspModuleId::Eq:        return "eq";
        case DspModuleId::Subsonic:  return "subsonic";
        case DspModuleId::Compander: return "comp";
        case DspModuleId::Limiter:   return "lim";
        default:                     return "";
    }
}
