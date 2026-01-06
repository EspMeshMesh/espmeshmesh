#include "wifi.h"
#include "log.h"

#include <memory.h>

#include "wifi_esp32.h"
#include "wifi_esp8266.h"

static const char *TAG = "espmeshmesh.wifi";

namespace espmeshmesh {

Wifi::Wifi(EspMeshMesh *mesh): mMesh(mesh) {
}

void Wifi::setup(std::string hostname, uint8_t channel, uint8_t txPower) {
    mHostname = hostname;
    mChannel = channel;
    mTxPower = txPower;

    if(mAesPassword.size() == 0) {
        mAesPassword = "1234567890ABCDEF";
    } else {
        mAesPassword = encryptPassword(mAesPassword);
    }
}

void Wifi::dump_config() {
    LIB_LOGCONFIG(TAG, "Wifi configuration:");
    LIB_LOGCONFIG(TAG, "Hostname: %s", mHostname.c_str());
    LIB_LOGCONFIG(TAG, "Channel: %d", mChannel);
    LIB_LOGCONFIG(TAG, "Tx Power: %d", mTxPower);
    LIB_LOGCONFIG(TAG, "Aes Password: %s", mAesPassword.c_str());
}

Wifi *wifiFactory(EspMeshMesh *mesh) {
#if defined(IDF_VER)
    return new WifiEsp32(mesh);
#elif defined(ESP8266)
    return new WifiEsp8266(mesh);
#else
    return nullptr;
#endif
}

}  // namespace espmeshmesh