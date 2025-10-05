#pragma once
#include "defines.h"

#ifdef USE_MULTIPATH_PROTOCOL

#include "packetbuf.h"
#include "recvdups.h"

#include <cstdint>
#include <functional>

namespace espmeshmesh {

typedef std::function<void(uint8_t *data, uint16_t size, uint32_t from, int16_t  rssi, uint8_t *path, uint8_t pathSize)> MultiPathReceiveRadioPacketHandler;

struct MultiPathHeaderSt {
	uint8_t protocol;
	uint8_t flags;
    uint8_t port;
	uint16_t seqno;
    uint8_t pathLength;
    uint8_t pathIndex;
	uint16_t dataLength;
    uint32_t sourceAddress;
    uint32_t trargetAddress;
} __attribute__ ((packed));
typedef struct MultiPathHeaderSt MultiPathHeader;

/**
 * @brief MultiPath sent status handler
 * @param status true if the packet has been sent correctly, false otherwise
 */
typedef std::function<void(bool status)> MultiPathSentStatusHandler;

class MultiPathPacket: public RadioPacket {
public:
	explicit MultiPathPacket(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) {}
public:
    virtual void allocClearData(uint16_t size);
    void allocClearData(uint16_t size, uint8_t pathlen);
public:
	MultiPathHeader *multipathHeader() { return (MultiPathHeader *)clearData(); }
    uint32_t getPathItem(uint8_t index) { return uint32FromBuffer(clearData()+sizeof(MultiPathHeaderSt)+(sizeof(uint32_t)*index)); }
    void setPathItem(uint32_t address, uint8_t index) { uint32toBuffer(clearData()+sizeof(MultiPathHeaderSt)+(sizeof(uint32_t)*index), address); }
    void setPayload(const uint8_t *payoad);
	uint8_t *unicastPayload() { return (uint8_t *)(clearData()+sizeof(MultiPathHeaderSt)); }
public:
    void setSentStatusHandler(MultiPathSentStatusHandler handler) { mSentStatusHandler = handler; }
    MultiPathSentStatusHandler sentStatusHandler() const { return mSentStatusHandler; }
    void notifySentStatusHandler(bool status) const { if(mSentStatusHandler) mSentStatusHandler(status); }
private:
    MultiPathSentStatusHandler mSentStatusHandler = nullptr;
};

struct MultiPathBindedPort_st {
    MultiPathReceiveRadioPacketHandler handler;
    uint16_t port;
};

typedef MultiPathBindedPort_st MultiPathBindedPort_t;

class MultiPath: public PacketBufProtocol {
public:
    enum Direction { Forward, Reverse };
	MultiPath(PacketBuf *pbuf);
    void setup() {}
    void loop();
    uint8_t send(MultiPathPacket *pkt, bool initHeader, MultiPathSentStatusHandler handler);
    uint8_t send(const uint8_t *data, uint16_t size, uint32_t target, uint32_t *path, uint8_t pathSize, MultiPath::Direction direction, uint8_t port, MultiPathSentStatusHandler handler);
    void receiveRadioPacket(uint8_t *p, uint16_t size, uint32_t f, int16_t  r);
    bool isPortAvailable(uint16_t port) const;
    bool bindPort(uint16_t port, MultiPathReceiveRadioPacketHandler h);
    void unbindPort(uint16_t port);
private:
    static void radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt);
    void radioPacketSent(uint8_t status, RadioPacket *pkt);
private:
    PacketBuf *packetbuf;
    void *mRecevieCallbackArg = nullptr;
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
private:
    std::list<MultiPathBindedPort_t> mBindedPorts;
};

} // namespace espmeshmesh

#endif