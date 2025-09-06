#pragma once

#include <stdint.h>
#include <cstddef>

namespace espmeshmesh {
    uint16_t crc_ibm_byte(uint16_t crc, const uint8_t c);
    uint16_t crc_ibm(uint16_t crc, uint8_t const *buffer, size_t len);
}