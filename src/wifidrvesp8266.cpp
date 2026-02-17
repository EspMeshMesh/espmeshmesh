#ifdef ESP8266

#include "wifidrvesp8266.h"
#include "log.h"

#include <Esp.h>
#include <osapi.h>
#include <user_interface.h>
#include "user_config.h"


static const char *TAG = "wifidrv_esp8266";

namespace espmeshmesh {

WifiDrvEsp8266 *wifiDrvEsp8266 = nullptr;


WifiDrvEsp8266::WifiDrvEsp8266() {
    wifiDrvEsp8266 = this;
}

WifiDrvEsp8266::~WifiDrvEsp8266() {
}

void WifiDrvEsp8266::driverSetup() {
    wifi_register_send_pkt_freedom_cb(this->_freedomCallback);
    system_os_task(recvTask_cb, PACKETBUF_TASK_PRIO, pktbufRecvTaskQueue, PACKETBUF_TASK_QUEUE_LEN);
}

}

void WifiDrvEsp8266::driverLoop() {
}

bool WifiDrvEsp8266::injectFrame(uint8_t *data, uint16_t len) {
    wifi_send_pkt_freedom(data, len, true);
    return true;
}

void WifiDrvEsp8266::freedomCallback(uint8_t status) {
    if(mInjectFrameCallback) {
        mInjectFrameCallback(status);
    }
}

void WifiDrvEsp8266::_freedomCallback(uint8_t status) {
    if(wifiDrvEsp8266) {
        wifiDrvEsp8266->freedomCallback(status);
    }
}

void WifiDrvEsp8266::recvTask(uint32_t index) {
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

        if(hwFreeHeap()<pktbufRecvTaskPacket[pktbufRecvTaskIndex].length+128 || hwFreeHeap()<MEMORY_TRESHOLD) {
            LIB_LOGE(TAG, "rawRecv low memory a:%d r:%ld", malloc_usable_size(pkt->payload), pktbufRecvTaskPacket[pktbufRecvTaskIndex].length);
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
#endif
#ifdef ESP8266
        system_os_post(PACKETBUF_TASK_PRIO, 0, pktbufRecvTaskIndex);
#endif
        if(++pktbufRecvTaskIndex >= PACKETBUF_TASK_QUEUE_LEN) pktbufRecvTaskIndex = 0;
    } else if(ieee80211_hdr->frame_control.Type == FRAME_TYPE_MANAGEMENT && ieee80211_hdr->frame_control.Subtype == FRAME_SUBTYPE_PROBE_REQUEST) {

    } else {
        //LIB_LOGD(TAG, "Unknow pkt type:%d sub:type:%d", ieee80211_hdr->frame_control.Type, ieee80211_hdr->frame_control.Subtype);
        //DEBUG_ARRAY("Unknow pkt: ", pkt->payload, pkt->rx_ctrl.sig_len);
    }
}

void WifiDrvEsp8266::_recvTask(ETSEvent *events) {
    if (events->sig == 0) {
      if(wifiDrvEsp8266) {
        wifiDrvEsp8266->recvTask((uint32_t)events->par);
      }
    }
  }
  
} // namespace espmeshmesh

#endif // ESP8266