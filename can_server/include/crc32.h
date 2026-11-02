#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t crc32_calc(const uint8_t* data, size_t len);
uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len);

#ifdef __cplusplus
}
#endif