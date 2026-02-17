#pragma once
#include "espmeshmesh.h"

#define PACKETBUF_TASK_QUEUE_LEN 12

namespace espmeshmesh {
class EspMeshMesh;

class WifiDrv {
    struct pktbuf_recvTask_packet_st {
        uint32_t length;
        uint8_t *data;
        int16_t rssi;
      };
      typedef struct pktbuf_recvTask_packet_st pktbuf_recvTask_packet_t;
public:
    WifiDrv();
    ~WifiDrv();

    const uint8_t *getAesPassword() const { return (uint8_t *) mAesPassword.c_str(); }
    void setAesPassword(std::string password) { mAesPassword = password; }

    virtual std::string encryptPassword(std::string password) = 0;

    void setup(std::string hostname, uint8_t channel, uint8_t txPower);
    void loop();
    virtual void dump_config();

    bool isLockdownModeActive() const { return this->mIsLockdownModeActive; }
    void setLockdownMode(bool active) { this->mIsLockdownModeActive = active; }

    virtual bool injectFrame(uint8_t *data, uint16_t len) = 0;
    void setInjectFrameCallback(std::function<void(uint8_t status)> callback);
    void setCaptureFrameCallback(std::function<void(uint8_t *data, uint16_t len, int16_t rssi)> callback);

protected:
    virtual void driverLoop() = 0;
    virtual void driverSetup() = 0;

protected:
    pktbuf_recvTask_packet_t pktbufRecvTaskPacket[PACKETBUF_TASK_QUEUE_LEN];
    uint32_t pktbufRecvTaskIndex;

private:
    bool mIsLockdownModeActive = false;
    std::string mHostname;
    uint8_t mChannel;
    uint8_t mTxPower;
    // Encryption password
    std::string mAesPassword;

    std::function<void(uint8_t status)> mInjectFrameCallback;
    std::function<void(uint8_t *data, uint16_t len, int16_t rssi)> mCaptureFrameCallback;
};

/*
 * @brief Create a new Wifi instance.
 * @return Wifi instance
 */
WifiDrv *wifiDrvFactory();

}
