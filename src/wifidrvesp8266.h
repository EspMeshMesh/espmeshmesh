#ifdef ESP8266
#pragma once
#include "wifidrv.h"

namespace espmeshmesh {

class WifiDrvEsp8266 : public WifiDrv {
    struct RxControl {
        signed rssi : 8;
        unsigned rate : 4;
        unsigned is_group : 1;
        unsigned : 1;
        unsigned sig_mode : 2;
        unsigned sig_len : 12;
        unsigned damatch0 : 1;
        unsigned damatch1 : 1;
        unsigned bssidmatch0 : 1;
        unsigned bssidmatch1 : 1;
        unsigned MCS : 7;
        unsigned CWB : 1;
        unsigned HT_length : 16;
        unsigned Smoothing : 1;
        unsigned Not_Sounding : 1;
        unsigned : 1;
        unsigned Aggregation : 1;
        unsigned STBC : 2;
        unsigned FEC_CODING : 1;
        unsigned SGI : 1;
        unsigned rxend_state : 8;
        unsigned ampdu_cnt : 8;
        unsigned channel : 4;
        unsigned : 12;
      };
      
      typedef struct RxPacket_st {
        struct RxControl rx_ctrl;
        uint8_t payload[];
      } RxPacket;
      
public:
    WifiDrvEsp8266();
    ~WifiDrvEsp8266();

    bool injectFrame(uint8_t *data, uint16_t len) override;
protected:
    void driverSetup() override;
    void driverLoop() override;
private:
    void freedomCallback(uint8_t status);
    static void _freedomCallback(uint8_t status);
    void recvTask(uint32_t index);
    static void _recvTask(ETSEvent *events);
private:
    os_event_t pktbufRecvTaskQueue[PACKETBUF_TASK_QUEUE_LEN];
};

}
#endif