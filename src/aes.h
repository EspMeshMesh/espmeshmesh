#pragma once
#include <stdint.h>

#define AES_BLOCK_SIZE 16


void * aes_encrypt_init(const uint8_t *key, size_t len);
void aes_encrypt(void *ctx, const uint8_t *plain, uint8_t *crypt);
void aes_encrypt_deinit(void *ctx);
void * aes_decrypt_init(const uint8_t *key, size_t len);
void aes_decrypt(void *ctx, const uint8_t *crypt, uint8_t *plain);
void aes_decrypt_deinit(void *ctx);

/**************************************************************************
 * AES declarations
 **************************************************************************/

/*

#define AES_MAXROUNDS			14
#define AES_BLOCKSIZE           16
#define AES_IV_SIZE             16

typedef struct aes_key_st {
    uint16_t rounds;
    uint16_t key_size;
    uint32_t ks[(AES_MAXROUNDS+1)*8];
    uint8_t iv[AES_IV_SIZE];
} MYAES_CTX;

typedef enum
{
    AES_MODE_128,
    AES_MODE_256
} AES_MODE;

void MYAES_set_key(MYAES_CTX *ctx, const uint8_t *key,
        const uint8_t *iv, AES_MODE mode);
void MYAES_cbc_encrypt(MYAES_CTX *ctx, const uint8_t *msg,
        uint8_t *out, int length);
void MYAES_cbc_decrypt(MYAES_CTX *ks, const uint8_t *in, uint8_t *out, int length);
void MYAES_convert_key(MYAES_CTX *ctx);
*/
