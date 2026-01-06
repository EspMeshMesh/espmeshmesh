#pragma once
#include "wifi.h"

namespace espmeshmesh {

class WifiEsp32 : public Wifi {
public:
    WifiEsp32(EspMeshMesh *mesh);
    void setup(std::string hostname, uint8_t channel, uint8_t txPower) override;
    std::string encryptPassword(std::string password) override;
private:
    void wifiInitMacAddr(uint8_t index);
private:
    bool setupWifiAp();
    bool setupWifiStation();      
};

}  // namespace espmeshmesh