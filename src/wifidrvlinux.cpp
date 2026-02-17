#ifdef USE_LINUX
#include "wifidrvlinux.h"
#include "log.h"

static const char *TAG = "wifidrv_linux";

namespace espmeshmesh {
WifiDrvLinux::WifiDrvLinux() {
}

WifiDrvLinux::~WifiDrvLinux() {
}

std::string WifiDrvLinux::encryptPassword(std::string password) {
    return password;
}

void WifiDrvLinux::driverSetup() {
}

void WifiDrvLinux::driverLoop() {
}

bool WifiDrvLinux::injectFrame(uint8_t *data, uint16_t len) {
    return true;
}
    
}
#endif