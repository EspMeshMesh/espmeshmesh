#include "encryption.h"
#include "defines.h"

#include <stddef.h>

#ifdef USE_ESP32
#include <esp_system.h>
#include <esp_random.h>
//#include <esp8266-compat.h>
#include <os.h>
#endif

#ifdef USE_ESP8266
#include <ets_sys.h>
#include <osapi.h>
#include <gpio.h>
#include <mem.h>
#endif

#ifdef USE_LINUX
#include <string.h>
extern "C" {
    #include <aes.h>
}
#endif

#define ENCRYPTION_BLOCK_LEN 16

namespace espmeshmesh {

struct AES_ctx encrypt_ctx;
struct AES_ctx decrypt_ctx;

void encryption_init(const uint8_t *key) {
#ifdef USE_LINUX
	AES_init_ctx(&encrypt_ctx, (uint8_t *)key);
	AES_init_ctx(&decrypt_ctx, (uint8_t *)key);
#else
if(!encrypt_ctx) encrypt_ctx = aes_encrypt_init((uint8_t *)key, keyLen);
if(!decrypt_ctx) decrypt_ctx = aes_decrypt_init((uint8_t *)key, keyLen);
#endif
}

void encrypt_data(uint8_t *dst, const uint8_t *src, uint16_t len) {
#ifndef USE_LINUX
	if(!encrypt_ctx) {
		return;
	}
#endif

	int i;
	uint8_t tmpsrc[AES_BLOCKLEN];
	uint8_t tmpdst[AES_BLOCKLEN];

	while(len>0) {
		uint8_t data_len = len > AES_BLOCKLEN ? AES_BLOCKLEN : len;
		if(data_len<AES_BLOCKLEN) {
			// Se il blocco e miniore di 16 aggingo un caratteri a case e il conado per ignorarli
			tmpsrc[data_len] = 0xFE;
			for(i=data_len+1;i<AES_BLOCKLEN;i++) tmpsrc[i] = (uint8_t)(espmeshmesh::random_uint32() & 0xFF);
		}

		AES_ECB_encrypt(&encrypt_ctx, tmpsrc);
		memcpy(dst, tmpsrc, AES_BLOCKLEN);

		dst += AES_BLOCKLEN;
		src += data_len;
		len -= data_len;
	}
}

void decrypt_data(uint8_t *dst, const uint8_t *src, uint16_t len) {
#ifndef USE_LINUX
	if(decrypt_ctx == 0 || len % AES_BLOCK_SIZE != 0) {
		return;
	}
#endif

	uint8_t tmpsrc[AES_BLOCKLEN];

	while(len>0) {
		memcpy(tmpsrc, src, AES_BLOCKLEN);
		AES_ECB_decrypt(&decrypt_ctx, tmpsrc);
		memcpy(dst, tmpsrc, AES_BLOCKLEN);

		dst += AES_BLOCKLEN;
		src += AES_BLOCKLEN;
		len -= AES_BLOCKLEN;
	}
}

}
