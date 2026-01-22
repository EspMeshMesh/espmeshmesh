#ifdef ESP8266
#include "wifi_esp8266.h"
#include "log.h"

#include <Esp.h>
#include <osapi.h>
#include <user_interface.h>
#include <md5.h>

static const char *TAG = "Wifi.Esp8266";

namespace espmeshmesh {

WifiEsp8266::WifiEsp8266(EspMeshMesh *mesh): Wifi(mesh) {
}

void WifiEsp8266::setup(std::string hostname, uint8_t channel, uint8_t txPower) {
    Wifi::setup(hostname, channel, txPower);
    LIB_LOGCONFIG(TAG, "Setting up meshmesh wifi...");
    wifi_station_set_hostname(hostname.c_str());
    wifi_set_opmode(STATION_MODE);
    wifiInitMacAddr(STATION_IF);
    wifi_station_set_auto_connect(false);
    wifi_set_phy_mode(PHY_MODE_11B);
    wifi_set_channel(channel);
    system_phy_set_max_tpw(txPower);
    LIB_LOGCONFIG(TAG, "Channel cfg:%d txPower:%d", channel, txPower);
    LIB_LOGD(TAG, "Wifi succesful!!!!");    
}

std::string WifiEsp8266::encryptPassword(std::string password) {
  md5_context_t md5;
  MD5Init(&md5);
  MD5Update(&md5, (uint8_t *) password.c_str(), password.size());
  uint8_t aespassword[16];
  MD5Final(aespassword, &md5);
  return std::string((char *) aespassword, 16);
}

void WifiEsp8266::wifiInitMacAddr(uint8_t index) {
  uint32_t id = Discovery::chipId();
  uint8_t *idptr = (uint8_t *) &id;
  uint8_t mac[6] = {0};
  mac[0] = 0xFE;
  mac[1] = 0x7F;
  mac[2] = idptr[3];
  mac[3] = idptr[2];
  mac[4] = idptr[1];
  mac[5] = idptr[0];
  LIB_LOGD(TAG, "wifiInitMacAddr %02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);

  wifi_set_macaddr(index, mac);
}

}  // namespace espmeshmesh
#endif