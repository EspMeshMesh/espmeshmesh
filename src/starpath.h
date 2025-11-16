#pragma once
#include "packetbuf.h"
#include "recvdups.h"

namespace espmeshmesh {

struct StarPathHeaderSt {
    uint8_t protocol;
    uint8_t flags;
    uint8_t port;
    uint16_t seqno;
} __attribute__ ((packed));
typedef struct StarPathHeaderSt StarPathHeader;

struct StarPathBindedPort_st {
    ReceiveHandler handler;
    uint16_t port;
};
typedef StarPathBindedPort_st StarPathBindedPort_t;

class StarPathPacket: public RadioPacket {
    explicit StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr): RadioPacket(owner, cb) {}
public:
    void allocClearData(uint16_t size) override;
    void allocClearData(uint16_t size, uint8_t pathlen);
};

class StarPath: public PacketBufProtocol {
    StarPath(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_STARPATH), mRecvDups() {}
    uint8_t send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    RecvDups mRecvDups;
};

}

