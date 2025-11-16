#pragma once
#include "packetbuf.h"
#include "recvdups.h"


#include <cstdint>

namespace espmeshmesh {

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

class MultiPathPacket: public RadioPacket {
public:
	explicit MultiPathPacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr): RadioPacket(owner, cb) {}
public:
    void allocClearData(uint16_t size) override;
    void allocClearData(uint16_t size, uint8_t pathlen);
public:
	MultiPathHeader *multipathHeader() { return (MultiPathHeader *)clearData(); }
    uint32_t getPathItem(uint8_t index) { return uint32FromBuffer(clearData()+sizeof(MultiPathHeaderSt)+(sizeof(uint32_t)*index)); }
    void setPathItem(uint32_t address, uint8_t index) { uint32toBuffer(clearData()+sizeof(MultiPathHeaderSt)+(sizeof(uint32_t)*index), address); }
    void setPayload(const uint8_t *payoad);
	uint8_t *unicastPayload() { return (uint8_t *)(clearData()+sizeof(MultiPathHeaderSt)); }
};

struct MultiPathBindedPort_st {
    ReceiveHandler handler;
    uint16_t port;
};

typedef MultiPathBindedPort_st MultiPathBindedPort_t;

class MultiPath: public PacketBufProtocol {
public:
    enum Direction { Forward, Reverse };
	MultiPath(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_MULTIPATH), mRecvDups() {}
    void loop() override;
    uint8_t send(MultiPathPacket *pkt, bool initHeader, SentStatusHandler handler = nullptr);
    uint8_t send(const uint8_t *data, uint16_t size, uint32_t target, uint32_t *path, uint8_t pathSize, bool pathRev, uint8_t port, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
};

} // namespace espmeshmesh

