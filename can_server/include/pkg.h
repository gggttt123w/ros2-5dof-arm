#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

struct pkg {
    std::vector<uint8_t> data;
    uint32_t crc = 0;
    size_t size = 0;
};
