#pragma once
#include "defines.h"

#ifdef USE_MULTIPATH_PROTOCOL

#include "packetbuf.h"
#include "recvdups.h"

#include <cstdint>
#include <functional>

namespace espmeshmesh {

typedef std::function<void(void *arg, uint8_t *data, uint16_t size, uint32_t from, int16_t  rssi, uint8_t *path, uint8_t pathSize)> MultiPathReceiveHandler;

struct MultiPathHeaderSt {
	uint8_t protocol;
	uint8_t flags;
	uint16_t seqno;
    uint8_t pathLength;
    uint8_t pathIndex;
	uint16_t dataLength;
    uint32_t sourceAddress;
    uint32_t trargetAddress;
} __attribute__ ((packed));
typedef struct MultiPathHeaderSt MultiPathHeader;

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
};

class MultiPath {
public:
	MultiPath(PacketBuf *pbuf): mRecvDups() { packetbuf = pbuf; packetbuf->setMultiPath(this); }
    void setup() {}
    void loop();
    uint8_t send(MultiPathPacket *pkt, bool initHeader);
    uint8_t send(const uint8_t *data, uint16_t size, uint8_t *target, uint8_t *path, uint8_t pathSize, bool pathRev);
    void receiveRadioPacket(uint8_t *p, uint16_t size, uint32_t f, int16_t  r);
    void setReceiveCallback(MultiPathReceiveHandler recvCb, void *arg);
private:
    static void radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt);
    void radioPacketSent(uint8_t status, RadioPacket *pkt);
private:
    PacketBuf *packetbuf;
    MultiPathReceiveHandler mRecevieCallback = nullptr;
    void *mRecevieCallbackArg = nullptr;
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
};

} // namespace espmeshmesh

#endif