#ifdef USE_LINUX
#pragma once
#include "wifidrv.h"
#include <string>
#include <thread>
#include <atomic>

namespace espmeshmesh {

class WifiDrvLinux : public WifiDrv {
public:
    WifiDrvLinux();
    ~WifiDrvLinux();

    std::string encryptPassword(std::string password) override;

    bool injectFrame(uint8_t *data, uint16_t len) override;

    void dump_config() override;

    void setInterface(const std::string& iface) { mInterface = iface; }
    const std::string& getInterface() const { return mInterface; }

protected:
    void driverSetup() override;
    void driverLoop() override;
    void driverShutdown() override;

private:
    void recvLoop();
    bool setMonitorMode();
    uint16_t getRadiotapLength(const uint8_t *data, size_t len) const;
    int8_t getRadiotapRssi(const uint8_t *data, size_t len) const;
    void getRadioMacAddress(uint8_t macAddress[6]) const;
    void setRadioMacAddress(const uint8_t macAddress[6]);
    void setRadioChannel(uint8_t channel);
    void setNetInterfaceUpOrDown(bool up);

    int mSocketFd = -1;
    uint8_t mSequence = 0;
    std::atomic<bool> mRunning{false};
};

}
#endif