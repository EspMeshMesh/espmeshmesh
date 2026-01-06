#pragma once
#include "espmeshmesh.h"

namespace espmeshmesh {

class Wifi {
public:
    Wifi(EspMeshMesh *mesh);
    virtual void setup(std::string hostname, uint8_t channel, uint8_t txPower);
    virtual void dump_config();
    const uint8_t *getAesPassword() const { return (uint8_t *) mAesPassword.c_str(); }
    void setAesPassword(std::string password) { mAesPassword = password; }
    virtual std::string encryptPassword(std::string password) = 0;
protected:
    EspMeshMesh *mMesh{nullptr};
    std::string mHostname;
    uint8_t mChannel;
    uint8_t mTxPower;
    // Encryption password
    std::string mAesPassword;
};

/*
 * @brief Create a new Wifi instance.
 * @return Wifi instance
 */
Wifi *wifiFactory(EspMeshMesh *mesh);

}  // namespace espmeshmesh