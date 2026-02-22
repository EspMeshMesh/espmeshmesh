#include "radiopacket.h"
#include "packetbufprotocol.h"
#include "log.h"
#include "defines.h"
#include "encryption.h"
#include "dot11.h"

#include <cstring>
#include "packetbuf.h"

#ifndef CRYPTO_LEN
#define CRYPTO_LEN(X) ((X + 0x0F) & ~0x0F)
#endif

namespace espmeshmesh {

static const char *TAG = "meshmesh_radiopacket";

RadioPacket::~RadioPacket() {
    if(mEncryptedData) delete mEncryptedData;
    if(mClearData) delete mClearData;
}

void RadioPacket::fromRawData(const uint8_t *buf, uint16_t size) {
    mClearDataSize = size;
    mClearData = new uint8_t[mClearDataSize];
	memcpy(clearData(), buf, size);
}

void RadioPacket::allocClearData(uint16_t size) {
    mClearDataSize = size;
    mClearData = new uint8_t[mClearDataSize];
}

void RadioPacket::allocAndCopyClearData(uint8_t *data, uint16_t size) {
    mClearDataSize = size;
    mClearData = new uint8_t[mClearDataSize];
    memcpy(mClearData, data, size);
}

bool RadioPacket::encryptClearData() {
    uint16_t csize = CRYPTO_LEN(mClearDataSize);
    if(mEncryptedData) delete mEncryptedData;
	mEncryptedData = new uint8_t[csize + PACKETBUF_80211_SIZE];
    if(!mEncryptedData) {
        LIB_LOGE(TAG, "RadioPacket::encryptClearData can't allocate %d bytes", csize);
        mEncryptedDataSize = 0;
        return false;
    } else {
	    mEncryptedDataSize = csize;
	    encrypt_data(ptrData(), mClearData, mClearDataSize);
        //LIB_LOGD(TAG, "RadioPacket::encryptClearData encrypted %d bytes", mEncryptedDataSize);
        //LIB_LOGD(TAG, "RadioPacket::encryptClearData clear data: %02X %02X %02X %02X %02X %02X", mClearData[0], mClearData[1], mClearData[2], mClearData[3], mClearData[4], mClearData[5]);
        //LIB_LOGD(TAG, "RadioPacket::encryptClearData encrypted data: %02X %02X %02X %02X %02X %02X", ptrData()[0], ptrData()[1], ptrData()[2], ptrData()[3], ptrData()[4], ptrData()[5]);
        return true;
    }
}

void RadioPacket::reportSentStatus(uint8_t status, RadioPacket *data) {
  if (this->mOwner)
    this->mOwner->radioPacketSent(status, data);
}

#if 0
void RadioPacket::sendFreedom() {
    if(mEncryptedData == nullptr) return;

    auto ieee80211_hdr = (ieee80211_hdr_p)ptr80211();
    LIB_LOGD(TAG, "sendFreedom type %d,%d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
    LIB_LOGD(TAG, "mac1 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr1[0], ieee80211_hdr->addr1[1], ieee80211_hdr->addr1[2], \
        ieee80211_hdr->addr1[3], ieee80211_hdr->addr1[4], ieee80211_hdr->addr1[5]);
    LIB_LOGD(TAG, "mac2 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr2[0], ieee80211_hdr->addr2[1], ieee80211_hdr->addr2[2], \
        ieee80211_hdr->addr2[3], ieee80211_hdr->addr2[4], ieee80211_hdr->addr2[5]);
    LIB_LOGD(TAG, "mac3 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr3[0], ieee80211_hdr->addr3[1], ieee80211_hdr->addr3[2], \
        ieee80211_hdr->addr3[3], ieee80211_hdr->addr3[4], ieee80211_hdr->addr3[5]);
}
#endif

void RadioPacket::fill80211(uint8_t *targetId, uint8_t *pktbufNodeIdPtr) {
    uint16_t seq_ctrl = 1;
    memset(ptr80211(), 0, sizeof(ieee80211_hdr_st));

	ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)ptr80211();
	ieee80211_hdr->frame_control.Protocol = 0;
	ieee80211_hdr->frame_control.Type = FRAME_TYPE_DATA;
	ieee80211_hdr->frame_control.Subtype = FRAME_SUBTYPE_DATA;
    ieee80211_hdr->frame_control.FromDS = 0;
    ieee80211_hdr->frame_control.ToDS = 0;
    // For some unknown reason when sending broadcast pacakts if frame control is set to 0 
    // the esp32 will add fromDS flags and esp8266 will to receive the bradcast packets.
    // this is a workaround to avoid this issue.
    ieee80211_hdr->frame_control.Retry = targetId ? 0 : 1;

    ieee80211_hdr->seq_ctrl = ++seq_ctrl;
	// Fill addresses with 0xFF
	memset(ieee80211_hdr->addr1, 0xFF, 18);
	// Target for unicast packet
	if(targetId) {
		ieee80211_hdr->addr1[0] = 0xFE;
		ieee80211_hdr->addr1[1] = 0x7F;
		ieee80211_hdr->addr1[2] = targetId[3];
		ieee80211_hdr->addr1[3] = targetId[2];
		ieee80211_hdr->addr1[4] = targetId[1];
		ieee80211_hdr->addr1[5] = targetId[0];
	} else {
  #ifdef USE_BROADCAST_WITH_MULTICAST
        ieee80211_hdr->addr1[0] = 0x01;
        ieee80211_hdr->addr1[1] = 0x00;
        ieee80211_hdr->addr1[2] = 0x5E;
        ieee80211_hdr->addr1[3] = 0x7F;
        ieee80211_hdr->addr1[4] = 0x00;
        ieee80211_hdr->addr1[5] = 0x01;
  #else
        ieee80211_hdr->addr1[0] = 0xFF;
        ieee80211_hdr->addr1[1] = 0xFF;
        ieee80211_hdr->addr1[2] = 0xFF;
        ieee80211_hdr->addr1[3] = 0xFF;
        ieee80211_hdr->addr1[4] = 0xFF;
        ieee80211_hdr->addr1[5] = 0xFF;
  #endif
    }

	// Source of target
	ieee80211_hdr->addr2[0] = 0xFE;
	ieee80211_hdr->addr2[1] = 0x7F;
	ieee80211_hdr->addr2[2] = pktbufNodeIdPtr[3];
	ieee80211_hdr->addr2[3] = pktbufNodeIdPtr[2];
	ieee80211_hdr->addr2[4] = pktbufNodeIdPtr[1];
	ieee80211_hdr->addr2[5] = pktbufNodeIdPtr[0];

	/*ieee80211_hdr->addr3[0] = 0x48;
	ieee80211_hdr->addr3[1] = 0x2C;
	ieee80211_hdr->addr3[2] = 0xA0;
	ieee80211_hdr->addr3[3] = 0x71;
	ieee80211_hdr->addr3[4] = 0x91;
	ieee80211_hdr->addr3[5] = 0x82; */
}

uint32_t RadioPacket::target8211() const {
    ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)ptr80211();
    if(ieee80211_hdr == nullptr) return 0;
    return uint32FromBuffer(ieee80211_hdr->addr1+2, true);
}

} // namespace espmeshmesh