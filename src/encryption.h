#pragma once
#include <stdint.h>

namespace espmeshmesh {

void encryption_init(const uint8_t *key);
void encrypt_data(uint8_t *dst, const uint8_t *src, uint16_t len);
void decrypt_data(uint8_t *dst, const uint8_t *src, uint16_t len);

}