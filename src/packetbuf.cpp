#include "packetbuf.h"

#include "log.h"
#include "defines.h"
#include "discovery.h"
#include "encryption.h"
#include "wifidrv.h"
#include "radiopacket.h"
#include "packetbufprotocol.h"

#include <cstring>

#define MEMORY_TRESHOLD 0x1000

namespace espmeshmesh {

static const char *TAG = "meshmesh_packetbuf";


PacketBuf *PacketBuf::singleton = nullptr;

PacketBuf *PacketBuf::getInstance() {
    PacketBuf *p =new PacketBuf();
    p->singleton = p;
    return p;
}

/**
 * Sends a radio packet using the freedom function.
 * If the packet buffer is busy, the packet is added to the queue.
 * The packet is sent when the buffer is free.
 * @param pkt The radio packet to send.
 */
void PacketBuf::send(RadioPacket *pkt) {
    if(!pktbufSent) {
        pktbufSent = pkt;
        if(mWifiDrv && pkt->encryptedData()) {
            mWifiDrv->injectFrame(pkt->ptr80211(), pkt->len80211());
        } else {
            LIB_LOGE(TAG, "send: no driver or encrypted data");
        }
    } else {
        // FIXME: Limit maximum queue size
        mPacketQueue.push_back(pkt);
    }
}

void PacketBuf::injectFrameCallback(uint8_t status) {
	if(pktbufSent) {
        pktbufSent->reportSentStatus(status, pktbufSent);
        if (pktbufSent->isAutoDelete()) {
            delete pktbufSent;
        }
        pktbufSent = nullptr;
	}

    if(mPacketQueue.size()>0) {
        pktbufSent = mPacketQueue.front();
        mPacketQueue.pop_front();
        if(mWifiDrv && pktbufSent->encryptedData()) {
            mWifiDrv->injectFrame(pktbufSent->ptr80211(), pktbufSent->len80211());
        } else {
            LIB_LOGE(TAG, "freedomCallback: no driver or encrypted data");
        }
    }
}

void PacketBuf::captureFrameCallback(const uint8_t *data, uint16_t len, int16_t rssi) {
    ieee80211_hdr_p ieee80211_hdr = (ieee80211_hdr_p)data;

    if (len < PACKETBUF_80211_SIZE + 5) {
        LIB_LOGE(TAG, "captureFrameCallback short packet");
        return;
    }

    lastpktLen = len - PACKETBUF_80211_SIZE - 4;
    int16_t lastpktRssi = rssi;

    if (hwFreeHeap() < lastpktLen + 128 || hwFreeHeap() < MEMORY_TRESHOLD) {
        LIB_LOGE(TAG, "captureFrameCallback low memory a:%d r:%d l:%d", hwFreeHeap(), lastpktLen, len);
        return;
    }

    uint8_t *clear = new uint8_t[lastpktLen];
    decrypt_data(clear, data + PACKETBUF_80211_SIZE, lastpktLen);

    uint32_t from;
    uint8_t *fromptr = (uint8_t *) &from;
    // Address in wifi packet is LE
    fromptr[0] = ieee80211_hdr->addr2[5];
    fromptr[1] = ieee80211_hdr->addr2[4];
    fromptr[2] = ieee80211_hdr->addr2[3];
    fromptr[3] = ieee80211_hdr->addr2[2];

    MeshAddress::DataSrc prot = (MeshAddress::DataSrc)clear[0];
    // LIB_LOGD(TAG, "recvTask sk %d len %d prot %d", index, pktbufRecvTaskPacket[index].length, prot);

    if (mRecvHandler.count(prot) > 0 && mRecvHandler[prot] != nullptr) {
      mRecvHandler[prot]->radioPacketRecv(clear, lastpktLen, from, lastpktRssi);
    } else {
      LIB_LOGVV(TAG, "Unknow protocol %d from %06X size %d %d %d", prot, from, lastpktLen,
                ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
      // DEBUG_ARRAY("Pay: ", pktbufRecvTaskPacket[events->par].data, pktbufRecvTaskPacket[events->par].length);
      LIB_LOGVV(TAG, "Bytes %02X %02X %02X %02X %02X %02X", clear[0], clear[1], clear[2], clear[3], clear[4], clear[5]);
    }

    delete[] clear;
}

void PacketBuf::setup(const uint8_t *aeskey) {
    mWifiDrv = EspMeshMesh::getInstance()->getWifiDrv();
    mWifiDrv->setCaptureFrameCallback(std::bind(&PacketBuf::captureFrameCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    mWifiDrv->setInjectFrameCallback(std::bind(&PacketBuf::injectFrameCallback, this, std::placeholders::_1));
    encryption_init(aeskey);
	pktbufNodeId = chipId();
	pktbufNodeIdPtr = (uint8_t *)&pktbufNodeId;
}

}  // namespace espmeshmesh
