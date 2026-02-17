#pragma once
#include "meshaddress.h"

#include <map>
#include <functional>
#include <cstdint>

/**
 * @brief PacketBufProtocol base class for all protocols
 */
namespace espmeshmesh {
class PacketBuf;
class RadioPacket;

typedef std::function<void(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi)> ProtocolReceiveHandler;

class PacketBufProtocol {
public:
    PacketBufProtocol(PacketBuf *pbuf, ProtocolReceiveHandler rx_fn = nullptr, MeshAddress::DataSrc protocol=MeshAddress::SRC_NONE);
    virtual ~PacketBufProtocol() {}
    virtual void setup() {};
    virtual void loop() {};
    virtual void shutdown() {};
    virtual bool teardown() { return true; };
    
    MeshAddress::DataSrc protocolType() const { return mProtocolType; }
    bool isPortAvailable(uint16_t port) const;
    bool bindPort(uint16_t port, ProtocolReceiveHandler handler);
    void unbindPort(uint16_t port);
    void callReceiveHandler(const uint8_t *payload, uint16_t size,const MeshAddress &from, int16_t rssi);
   
    virtual void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) = 0;
    virtual void radioPacketSent(uint8_t status, RadioPacket *pkt);
    
protected:
    PacketBuf *mPacketBuf{nullptr};
    std::map<uint16_t, ProtocolReceiveHandler> mBindedPorts;
    MeshAddress::DataSrc mProtocolType = MeshAddress::SRC_NONE;
};

}  // namespace espmeshmesh