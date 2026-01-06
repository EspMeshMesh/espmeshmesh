#pragma once
#include "wifi.h"

namespace espmeshmesh {

class WifiEsp8266 : public Wifi {
public:
    WifiEsp8266(EspMeshMesh *mesh);
    void setup(std::string hostname, uint8_t channel, uint8_t txPower) override;
    std::string encryptPassword(std::string password) override;
private:
    void wifiInitMacAddr(uint8_t index);
};

}  // namespace espmeshmesh