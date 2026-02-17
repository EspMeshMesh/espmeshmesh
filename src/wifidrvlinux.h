#ifdef USE_LINUX
#pragma once
#include "wifidrv.h"

namespace espmeshmesh {

class WifiDrvLinux : public WifiDrv {
public:
    WifiDrvLinux();
    ~WifiDrvLinux();

    std::string encryptPassword(std::string password) override;

    bool injectFrame(uint8_t *data, uint16_t len) override;
protected:
    void driverSetup() override;
    void driverLoop() override;
private:
    void wifiTask(void *arg);
    void recvLoop();
    void sendByte(uint8_t data);
};

}
#endif