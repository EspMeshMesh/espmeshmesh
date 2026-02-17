#include "wifidrv.h"
#include "espmeshmesh.h"
#include "wifidrvesp32.h"
#include "wifidrvesp8266.h"
#include "wifidrvlinux.h"

namespace espmeshmesh {

WifiDrv::WifiDrv() {
}

WifiDrv::~WifiDrv() {
}

void WifiDrv::setup(std::string hostname, uint8_t channel, uint8_t txPower) {
    /*int i;
    pktbufRecvTaskIndex = 0;
    for(i=0; i< PACKETBUF_TASK_QUEUE_LEN;    i++) {
        pktbufRecvTaskPacket[i].data = 0;
        pktbufRecvTaskPacket[i].length = 0;
    }*/

    mHostname = hostname;
    mChannel = channel;
    mTxPower = txPower;

    if(mAesPassword.size() == 0) {
        mAesPassword = "1234567890ABCDEF";
    } else {
        mAesPassword = encryptPassword(mAesPassword);
    }
}

void WifiDrv::loop() {
}

void WifiDrv::dump_config() {
    LIB_LOGCONFIG(TAG, "Wifi configuration:");
    LIB_LOGCONFIG(TAG, "Hostname: %s", mHostname.c_str());
    LIB_LOGCONFIG(TAG, "Channel: %d", mChannel);
    LIB_LOGCONFIG(TAG, "Tx Power: %d", mTxPower);
    LIB_LOGCONFIG(TAG, "Aes Password: %s", mAesPassword.c_str());
}

void WifiDrv::setInjectFrameCallback(std::function<void(uint8_t status)> callback) {
    mInjectFrameCallback = callback;
}

void WifiDrv::setCaptureFrameCallback(std::function<void(uint8_t *data, uint16_t len, int16_t rssi)> callback) {
    mCaptureFrameCallback = callback;
}

WifiDrv *wifiDrvFactory() {
    #if defined(IDF_VER)
        return new WifiDrvEsp32();
    #elif defined(ESP8266)
        return new WifiDrvEsp8266();
    #elif defined(USE_LINUX)
        return new WifiDrvLinux();
    #else
        return nullptr;
    #endif
}

} // namespace espmeshmesh