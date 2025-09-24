#include "packetbuf.h"

#include "log.h"
#include "defines.h"
#include "discovery.h"
#include "broadcast.h"
#include "broadcast2.h"
#include "unicast.h"
#ifdef USE_MULTIPATH_PROTOCOL
#include "multipath.h"
#endif
#ifdef USE_POLITE_BROADCAST_PROTOCOL
#include "polite.h"
#endif
#ifdef USE_CONNECTED_PROTOCOL
#include "connectedpath.h"
#endif

#ifdef ESP8266
#include <Esp.h>
#include <osapi.h>
#include <user_interface.h>
#include "user_config.h"
#endif

#ifdef IDF_VER
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>
#include <freertos/queue.h>
#endif

extern "C" {
    #include "encryption.h"
}

#define FRAME_TYPE_MANAGEMENT 0
#define FRAME_TYPE_CONTROL 1
#define FRAME_TYPE_DATA 2
#define FRAME_SUBTYPE_DATA 0
#define FRAME_SUBTYPE_PROBE_REQUEST 0x04
#define FRAME_SUBTYPE_PROBE_RESPONSE 0x05
#define FRAME_SUBTYPE_BEACON 0x08
#define FRAME_SUBTYPE_AUTH 0x0b
#define FRAME_SUBTYPE_DEAUTH 0x0c

#define MEMORY_TRESHOLD 0x1000

namespace espmeshmesh {

static const char *TAG = "meshmesh_packetbuf";

struct framectrl_80211_st {
    uint8_t Protocol:2;
    uint8_t Type:2;
    uint8_t Subtype:4;
    uint8_t ToDS:1;
    uint8_t FromDS:1;
    uint8_t MoreFlag:1;
    uint8_t Retry:1;
    uint8_t PwrMgmt:1;
    uint8_t MoreData:1;
    uint8_t Protectedframe:1;
    uint8_t Order:1;
};

typedef struct framectrl_80211_st framectrl_80211_t;
typedef framectrl_80211_t *framectrl_80211_p;

struct ieee80211_hdr_st {
	framectrl_80211_t frame_control;
	uint16_t duration_id;
	uint8_t addr1[6];
	uint8_t addr2[6];
	uint8_t addr3[6];
	uint16_t seq_ctrl;
	//uint8_t addr4[6];
} __attribute__ ((packed));

typedef struct ieee80211_hdr_st ieee80211_hdr_t;

typedef ieee80211_hdr_t *ieee80211_hdr_p;

#define PACKETBUF_LEN_PINDEX		2
#define PACKETBUF_LEN_PINDEX		2

#ifndef CRYPTO_LEN
#define CRYPTO_LEN(X) {}
#endif

#ifdef IDF_VER
void promiscuousRxq(void* buf, wifi_promiscuous_pkt_type_t type);
#endif

RadioPacket::~RadioPacket() {
    if(mEncryptedData) delete mEncryptedData;
    if(mClearData) delete mClearData;
}

void RadioPacket::fromRawData(uint8_t *buf, uint16_t size) {
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
        return true;
    }
}

void RadioPacket::callCallback(uint8_t status, RadioPacket *data) {
    if(mCallbackArg) mCallback(mCallbackArg, status, data);
}

void RadioPacket::sendFreedom() {
    if(mEncryptedData == nullptr) return;

#if 0
    auto ieee80211_hdr = (ieee80211_hdr_p)ptr80211();
    LIB_LOGD(TAG, "sendFreedom type %d,%d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
    LIB_LOGD(TAG, "mac1 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr1[0], ieee80211_hdr->addr1[1], ieee80211_hdr->addr1[2], \
        ieee80211_hdr->addr1[3], ieee80211_hdr->addr1[4], ieee80211_hdr->addr1[5]);
    LIB_LOGD(TAG, "mac2 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr2[0], ieee80211_hdr->addr2[1], ieee80211_hdr->addr2[2], \
        ieee80211_hdr->addr2[3], ieee80211_hdr->addr2[4], ieee80211_hdr->addr2[5]);
    LIB_LOGD(TAG, "mac3 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr3[0], ieee80211_hdr->addr3[1], ieee80211_hdr->addr3[2], \
        ieee80211_hdr->addr3[3], ieee80211_hdr->addr3[4], ieee80211_hdr->addr3[5]);
#endif

#ifdef ESP8266
    wifi_send_pkt_freedom(ptr80211(), len80211(), true);
#endif

#ifdef USE_ESP32
    //LIB_LOGD(TAG, "sendFreedom esp_wifi_80211_tx about to send %d bytes", len80211());
    esp_err_t res = esp_wifi_80211_tx(WIFI_IF_STA, ptr80211(), len80211(), true);
    if(res != ESP_OK) {
        LIB_LOGE(TAG, "sendFreedom esp_wifi_80211_tx err %d", res);
    }
#endif
}

void RadioPacket::fill80211(uint8_t *targetId, uint8_t *pktbufNodeIdPtr) {
    uint16_t seq_ctrl = 1;
    memset(ptr80211(), 0, sizeof(ieee80211_hdr_st));

	ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)ptr80211();
	ieee80211_hdr->frame_control.Protocol = 0;
	ieee80211_hdr->frame_control.Type = FRAME_TYPE_DATA;
	ieee80211_hdr->frame_control.Subtype = FRAME_SUBTYPE_DATA;

#ifdef USE_ESP32
  ieee80211_hdr->frame_control.FromDS = targetId ? 0 : 1;
  ieee80211_hdr->frame_control.ToDS = 0;
#endif

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

PacketBuf *PacketBuf::singleton = nullptr;

PacketBuf *PacketBuf::getInstance() {
    PacketBuf *p =new PacketBuf();
    p->singleton = p;
    return p;
}

uint8_t PacketBuf::send(RadioPacket *pkt) {
    if(!pktbufSent) {
        pktbufSent = pkt;
        pktbufSent->sendFreedom();
    } else {
        // FIXME: Limit maximum queue size
        mPacketQueue.push_back(pkt);
    }
	return PKT_SEND_OK;
}

void PacketBuf::freedomCallback(uint8_t status) {
	if(pktbufSent) {
        pktbufSent->callCallback(status, pktbufSent);
        if(pktbufSent->isAutoDelete()) delete pktbufSent;
        pktbufSent = nullptr;
	}

    if(mPacketQueue.size()>0) {
        pktbufSent = mPacketQueue.front();
        mPacketQueue.pop_front();
        pktbufSent->sendFreedom();
    }
}

#ifdef IDF_VER
void PacketBuf::recvTask(uint32_t index) {
    {
#else
void PacketBuf::recvTask_cb(os_event_t *events) {
    if(singleton) {
        singleton->recvTask(events);
    }
}

void PacketBuf::recvTask(os_event_t *events) {
    if(events->sig == 0) {
        uint32_t index = events->par;
#endif
        ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)pktbufRecvTaskPacket[index].data;

        if(pktbufRecvTaskPacket[index].length < PACKETBUF_80211_SIZE+5) {
            LIB_LOGE(TAG, "recvTask short packet");
            return;
        }

        lastpktLen = pktbufRecvTaskPacket[index].length - PACKETBUF_80211_SIZE - 4;
        int16_t lastpktRssi = pktbufRecvTaskPacket[index].rssi;

#ifdef IDF_VER
        if(heap_caps_get_free_size(MALLOC_CAP_8BIT)<lastpktLen+128 || heap_caps_get_free_size(MALLOC_CAP_8BIT)<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "recvTask low memory a:%d r:%d l:%ld", heap_caps_get_free_size(MALLOC_CAP_8BIT), lastpktLen, pktbufRecvTaskPacket[index].length);
#else
        if(ESP.getMaxFreeBlockSize()<lastpktLen+128 || ESP.getMaxFreeBlockSize()<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "recvTask low memory a:%d r:%d l:%d", ESP.getMaxFreeBlockSize(), lastpktLen, pktbufRecvTaskPacket[index].length);
#endif
            return;
        }

        uint8_t *clear = new uint8_t[lastpktLen];
        decrypt_data(clear, pktbufRecvTaskPacket[index].data + PACKETBUF_80211_SIZE, lastpktLen);

        uint32_t from; uint8_t *fromptr = (uint8_t *)&from;
        // Address in wifi packet is LE
        fromptr[0] = ieee80211_hdr->addr2[5];
        fromptr[1] = ieee80211_hdr->addr2[4];
        fromptr[2] = ieee80211_hdr->addr2[3];
        fromptr[3] = ieee80211_hdr->addr2[2];

        uint8_t prot = clear[0];
        //LIB_LOGD(TAG, "recvTask sk %d len %d prot %d", index, pktbufRecvTaskPacket[index].length, prot);

        switch(prot) {
        case PROTOCOL_BROADCAST:
            if(broadcast) broadcast->recv(clear, lastpktLen, fromptr, lastpktRssi);
            break;
        case PROTOCOL_BROADCAST_V2:
            if(broadcast2) broadcast2->recv(clear, lastpktLen, from, lastpktRssi);
            break;
        case PROTOCOL_UNICAST:
            if(unicast) unicast->receiveRadioPacket(clear, lastpktLen, from, lastpktRssi);
            break;
        case PROTOCOL_MULTIPATH:
#ifdef USE_MULTIPATH_PROTOCOL
            if(multipath) multipath->receiveRadioPacket(clear, lastpktLen, from, lastpktRssi);
#endif
            break;
        case PROTOCOL_POLITEBRD:
#ifdef USE_POLITE_BROADCAST_PROTOCOL
            if(mPoliteBroadcast) mPoliteBroadcast->receiveRadioPacket(clear, lastpktLen, fromptr, lastpktRssi);
#endif
            break;
        case PROTOCOL_CONNPATH:
#ifdef USE_CONNECTED_PROTOCOL
            if(mConnectedPath) mConnectedPath->receiveRadioPacket(clear, lastpktLen, from, lastpktRssi);
#endif
            break;
        default:
            LIB_LOGVV(TAG, "Unknow protocol %d from %06X size %d %d %d", prot, from, lastpktLen, ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
            //DEBUG_ARRAY("Pay: ", pktbufRecvTaskPacket[events->par].data, pktbufRecvTaskPacket[events->par].length);
            LIB_LOGVV(TAG, "Bytes %02X %02X %02X %02X %02X %02X", clear[0], clear[1], clear[2], clear[3], clear[4], clear[5]);
        }

        delete []clear;
        delete pktbufRecvTaskPacket[index].data;
        memset(&pktbufRecvTaskPacket[index], 0, sizeof(pktbuf_recvTask_packet_t));
    }
}

#ifdef IDF_VER
void PacketBuf::wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus) {
    //LIB_LOGD(TAG, "wifiTxDoneCb if:%d sent:%d len:%d", ifidx, txStatus, *data_len);
    if(singleton) singleton->freedomCallback(txStatus?0:1);
}
#else
void PacketBuf::freedomCallback_cb(uint8_t status) {
    if(singleton) {
        singleton->freedomCallback(status);
    }
}
#endif

void PacketBuf::setup(const uint8_t *aeskey, int aeskeylen) {
    int i;
    pktbufRecvTaskIndex = 0;
#ifdef IDF_VER
    esp_err_t ret;
    if((ret = esp_wifi_set_promiscuous_rx_cb(promiscuousRxq)) != ESP_OK) {
        LIB_LOGE(TAG, "esp_wifi_set_promiscuous_rx_cb error %d", ret);
    }
    mRecvQueue = xQueueCreate(16,sizeof(uint32_t));
#else
    system_os_task(recvTask_cb, PACKETBUF_TASK_PRIO, pktbufRecvTaskQueue, PACKETBUF_TASK_QUEUE_LEN);
#endif
    encryption_init(aeskey, aeskeylen);
    for(i=0; i< PACKETBUF_TASK_QUEUE_LEN;    i++) {
        pktbufRecvTaskPacket[i].data = 0;
        pktbufRecvTaskPacket[i].length = 0;
    }

	pktbufNodeId = Discovery::chipId();
	pktbufNodeIdPtr = (uint8_t *)&pktbufNodeId;
#ifdef IDF_VER
    esp_wifi_set_tx_done_cb(wifiTxDoneCb);
#else
	wifi_register_send_pkt_freedom_cb(freedomCallback_cb);
#endif
}

#ifdef IDF_VER
void PacketBuf::loop() {
    uint32_t index;
    while(xQueueReceive(mRecvQueue, &index, 0)) {
        recvTask(index);
    }
}
#endif

void IRAM_ATTR __attribute__((hot)) PacketBuf::rawRecv(RxPacket *pkt) {
  if(isLockdownModeActive) {
    return;
  }

    ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)pkt->payload;

#if 0
    if(ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_DATA) {
        LIB_LOGD(TAG, "rawRecv type %d,%d len %d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype, pkt->rx_ctrl.sig_len);
        LIB_LOGD(TAG, "mac1 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr1[0], ieee80211_hdr->addr1[1], ieee80211_hdr->addr1[2], \
            ieee80211_hdr->addr1[3], ieee80211_hdr->addr1[4], ieee80211_hdr->addr1[5]);
        LIB_LOGD(TAG, "mac2 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr2[0], ieee80211_hdr->addr2[1], ieee80211_hdr->addr2[2], \
            ieee80211_hdr->addr2[3], ieee80211_hdr->addr2[4], ieee80211_hdr->addr2[5]);
        LIB_LOGD(TAG, "mac3 %02X:%02X:%02X:%02X:%02X:%02X", ieee80211_hdr->addr3[0], ieee80211_hdr->addr3[1], ieee80211_hdr->addr3[2], \
            ieee80211_hdr->addr3[3], ieee80211_hdr->addr3[4], ieee80211_hdr->addr3[5]);
    }
#endif

#ifdef IDF_VER
    static uint8_t brdaddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if(memcmp(brdaddr, ieee80211_hdr->addr1, 6) != 0 && (ieee80211_hdr->addr1[0]!=0xFE || ieee80211_hdr->addr1[1]!=0x7F ||
        ieee80211_hdr->addr1[2]!=nodeIdPtr()[3] || ieee80211_hdr->addr1[3]!=nodeIdPtr()[2] || ieee80211_hdr->addr1[4]!=nodeIdPtr()[1],
        ieee80211_hdr->addr1[5]!=nodeIdPtr()[0])) {
        return;
    }
#endif

    if(ieee80211_hdr->frame_control.Type == FRAME_TYPE_DATA && ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_DATA && \
            ieee80211_hdr->addr2[0] == 0xFE && (ieee80211_hdr->addr2[1]&0xF0) == 0x70) {

        //LIB_LOGD(TAG, "rawRecv type %d,%d len %d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype, pkt->rx_ctrl.sig_len);
        if(pkt->rx_ctrl.sig_len<=PACKETBUF_80211_SIZE+5) {
            LIB_LOGE(TAG, "rawRecv short packet %d", pkt->rx_ctrl.sig_len);
            return;
        }

#ifdef IDF_VER
        if(heap_caps_get_free_size(MALLOC_CAP_8BIT)<pktbufRecvTaskPacket[pktbufRecvTaskIndex].length+128 || heap_caps_get_free_size(MALLOC_CAP_8BIT)<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "recvTask low memory a:%d r:%ld", heap_caps_get_free_size(MALLOC_CAP_8BIT), pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
#else
        if(ESP.getMaxFreeBlockSize()<pktbufRecvTaskPacket[pktbufRecvTaskIndex].length+128 || ESP.getMaxFreeBlockSize()<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "rawRecv low memory a:%d r:%d", ESP.getMaxFreeBlockSize(), pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
#endif
            return;
        }

        if(pktbufRecvTaskPacket[pktbufRecvTaskIndex].length > 0) {
            LIB_LOGE(TAG, "rawRecv fifo full");
            return;
        }

        //LIB_LOGD(TAG, "rawRecv enqueued %d len %d", pktbufRecvTaskIndex, pkt->rx_ctrl.sig_len);
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].length = pkt->rx_ctrl.sig_len;
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].data = new uint8_t[pktbufRecvTaskPacket[pktbufRecvTaskIndex].length];
        memcpy(pktbufRecvTaskPacket[pktbufRecvTaskIndex].data, pkt->payload, pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
        pktbufRecvTaskPacket[pktbufRecvTaskIndex].rssi = pkt->rx_ctrl.rssi;
#ifdef IDF_VER
        xQueueSend(mRecvQueue, &pktbufRecvTaskIndex, 0);
#else
        system_os_post(PACKETBUF_TASK_PRIO, 0, pktbufRecvTaskIndex);
#endif
        if(++pktbufRecvTaskIndex >= PACKETBUF_TASK_QUEUE_LEN) pktbufRecvTaskIndex = 0;
    } else if(ieee80211_hdr->frame_control.Type == FRAME_TYPE_MANAGEMENT && ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_PROBE_REQUEST) {

    } else {
        //LIB_LOGD(TAG, "Unknow pkt type:%d sub:type:%d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
        //DEBUG_ARRAY("Unknow pkt: ", pkt->payload, pkt->rx_ctrl.sig_len);
    }
}

#ifdef IDF_VER
void promiscuousRxq(void* buf, wifi_promiscuous_pkt_type_t type) {
    //wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t*)buf;
    if(type == WIFI_PKT_DATA && PacketBuf::singleton) PacketBuf::singleton->rawRecv((RxPacket *)buf);
}
#endif



}  // namespace espmeshmesh
