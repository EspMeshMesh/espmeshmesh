#include "encryption.h"
#include <stddef.h>

#include "aes.h"

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

#define ENCRYPTION_BLOCK_LEN 16

static void *encrypt_ctx = 0;
static void *decrypt_ctx = 0;

void encryption_init(const uint8_t *key, uint8_t keyLen) {
	if(!encrypt_ctx) encrypt_ctx = aes_encrypt_init((uint8_t *)key, keyLen);
	if(!decrypt_ctx) decrypt_ctx = aes_decrypt_init((uint8_t *)key, keyLen);
}

void encrypt_data(uint8_t *dst, uint8_t *src, uint16_t len) {
	if(encrypt_ctx == 0) {
		return;
	}

	int i;
	uint8_t tmpsrc[AES_BLOCK_SIZE];
	uint8_t tmpdst[AES_BLOCK_SIZE];

	while(len>0) {
		uint8_t data_len = len > AES_BLOCK_SIZE ? AES_BLOCK_SIZE : len;
		if(data_len < AES_BLOCK_SIZE) memset(tmpsrc , 0, AES_BLOCK_SIZE);
		memcpy(tmpsrc, src, data_len);
		if(data_len<AES_BLOCK_SIZE) {
			// Se il blocco e miniore di 16 aggingo un caratteri a case e il conado per ignorarli
			tmpsrc[data_len] = 0xFE;
#ifdef USE_ESP32
			for(i=data_len+1;i<AES_BLOCK_SIZE;i++) tmpsrc[i] = (uint8_t)(esp_random() & 0xFF);
#else
			for(i=data_len+1;i<AES_BLOCK_SIZE;i++) tmpsrc[i] = (uint8_t)(phy_get_rand() & 0xFF);
#endif
		}
		aes_encrypt(encrypt_ctx, tmpsrc, tmpdst);
		os_memcpy(dst, tmpdst, AES_BLOCK_SIZE);

		dst += data_len;
		src += data_len;
		len -= data_len;
	}
}

void decrypt_data(uint8_t *dst, uint8_t *src, uint16_t len) {
	if(decrypt_ctx == 0) {
		return;
	}

	uint8_t tmpsrc[AES_BLOCK_SIZE];
	uint8_t tmpdst[AES_BLOCK_SIZE];

	while(len>0) {
		uint8_t data_len = len > AES_BLOCK_SIZE ? AES_BLOCK_SIZE : len;
		if(data_len < AES_BLOCK_SIZE) os_memset(tmpsrc , 0, AES_BLOCK_SIZE);
		os_memcpy(tmpsrc, src, data_len);
		aes_decrypt(decrypt_ctx, tmpsrc, tmpdst);
		os_memcpy(dst, tmpdst, data_len);

		dst += data_len;
		src += data_len;
		len -= data_len;
	}
}
