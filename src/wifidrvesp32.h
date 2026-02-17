#ifdef IDF_VER
#pragma once
#include "wifidrv.h"

namespace espmeshmesh {

class WifiDrvEsp32 : public WifiDrv {
public:
    WifiDrvEsp32();
    ~WifiDrvEsp32();

    bool injectFrame(uint8_t *data, uint16_t len) override;
protected:
    std::string encryptPassword(std::string password) override;
protected:
    void driverSetup();
    void driverLoop() override;
private:
    
private:
    void wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);
    static void _wifiTxDoneCb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);
    void promiscuousRxq(RxPacket *pkt);
    static void _promiscuousRxq(void* buf, wifi_promiscuous_pkt_type_t type);
private:
    QueueHandle_t mRecvQueue;
};

}

#endif // IDF_VER