#pragma once

#include <functional>
#include <cstdint>
#include "dot11.h"

namespace espmeshmesh {

class RadioPacket;
class PacketBufProtocol;
typedef std::function<void(int8_t status, RadioPacket *pkt)> SentStatusHandler;

class RadioPacket {
public:
    explicit RadioPacket(PacketBufProtocol *owner, SentStatusHandler cb) { this->mOwner = owner; this->mCallback = std::move(cb); }
    virtual ~RadioPacket();
    bool isAutoDelete() const { return mAutoDelete; }
    bool isBroadcast() const { return mIsBroadcast; }
    void setAutoDelete(bool autodel) { mAutoDelete = autodel; }
    
    void fromRawData(const uint8_t *buf, uint16_t size);
    
public:
    virtual void allocClearData(uint16_t size);
    void allocAndCopyClearData(uint8_t *data, uint16_t size);
    uint16_t clearDataSize() const { return mClearDataSize; }
    uint8_t *clearData() { return mClearData; }
    const uint8_t *clearData() const { return mClearData; }
    bool encryptClearData();
    
public:
    uint16_t encryptedDataSize() const { return mEncryptedDataSize; }
    uint8_t *encryptedData() const { return mEncryptedData; }
    
    void reportSentStatus(uint8_t status, RadioPacket *data);
    void setCallback(SentStatusHandler cb) { this->mCallback = std::move(cb); }
    SentStatusHandler getCallback() { return this->mCallback; }
    void callCallback(uint8_t status, RadioPacket *data) { if (this->mCallback) this->mCallback(status, data); }

public:
    // pktbuf_header_t *header() { return (pktbuf_header_t *)mEncryptedData; }
    uint8_t *ptr80211() const { return (uint8_t *) mEncryptedData; }
    uint16_t len80211() const { return mEncryptedDataSize + PACKETBUF_80211_SIZE; }
    void fill80211(uint8_t *targetId, uint8_t *pktbufNodeIdPtr);
    uint32_t target8211() const;
    uint8_t *ptrData() const { return (uint8_t *) (mEncryptedData + PACKETBUF_80211_SIZE); }
    
protected:
    void setIsBroadcast() { mIsBroadcast = true; }
    
private:
    SentStatusHandler mCallback{nullptr};
    PacketBufProtocol *mOwner{nullptr};
    // Full encrypted 802.11 packet
    uint8_t *mEncryptedData = nullptr;
    // Size of  encrypted data
    uint16_t mEncryptedDataSize = 0;
    // Clear payload that will be encrypted in finnal radio packet
    uint8_t *mClearData = nullptr;
    // Size of clear payload data
    uint16_t mClearDataSize = 0;
    // Autodelete packet after been sent
    bool mAutoDelete = true;
    // Is a broadcast packet
    bool mIsBroadcast = false;
};
    
} // namespace espmeshmesh