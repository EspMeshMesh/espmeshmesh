#pragma once
#include <stdint.h>

#define CRYPTO_BLOCK_SIZE 16
#define CRYPTO_BLOCK_MASK 0x000F
#define CRYPTO_BLOCK_NMASK 0xFFF0

// Padding a CRYPTO_BLOCK_SIZE bytes
#define CRYPTO_LEN(X) ((X&CRYPTO_BLOCK_MASK) == 0 ? X : (X&CRYPTO_BLOCK_NMASK) + CRYPTO_BLOCK_SIZE)

void encryption_init(const uint8_t *key, uint8_t keyLen);
void encrypt_data(uint8_t *dst, uint8_t *src, uint16_t len);
void decrypt_data(uint8_t *dst, uint8_t *src, uint16_t len);
