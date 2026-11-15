#include <stdint.h>

#define DR_FLAC_IMPLEMENTATION
#define DR_FLAC_NO_STDIO
#define DR_FLAC_NO_CRC
#define DR_FLAC_BUFFER_SIZE 4096
#define DRFLAC_API static
#define DRFLAC_PRIVATE static
#include "../../src/dr_flac.h"

int32_t test_drflac_pred32(uint32_t order, int32_t shift, const int32_t* coeffs, int32_t* decodedSamples) {
    return (int32_t)drflac__calculate_prediction_32(
        (drflac_uint32)order,
        (drflac_int32)shift,
        (const drflac_int32*)coeffs,
        (drflac_int32*)decodedSamples);
}

int32_t test_drflac_pred64(uint32_t order, int32_t shift, const int32_t* coeffs, int32_t* decodedSamples) {
    return (int32_t)drflac__calculate_prediction_64(
        (drflac_uint32)order,
        (drflac_int32)shift,
        (const drflac_int32*)coeffs,
        (drflac_int32*)decodedSamples);
}

uint32_t test_drflac_clz(uint32_t x) {
    return (uint32_t)drflac__clz((drflac_cache_t)x);
}

uint32_t test_drflac_ilog2(uint32_t x) {
    return (uint32_t)drflac__ilog2_u32((drflac_uint32)x);
}
