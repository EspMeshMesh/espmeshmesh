#pragma once
#include "packetbuf.h"
#include "recvdups.h"

#include <cstdint>
#include <functional>

namespace espmeshmesh {

struct UnicastHeaderSt {
	uint8_t protocol;
	uint8_t seqno;
	uint8_t flags;
	uint8_t port;
	uint16_t lenght;
} __attribute__ ((packed));
typedef struct UnicastHeaderSt UnicastHeader;

class UnicastPacket: public RadioPacket {
public:
	explicit UnicastPacket(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) {}
public:
    virtual void allocClearData(uint16_t size);
public:
	UnicastHeader *unicastHeader() { return (UnicastHeader *)clearData(); }
	uint8_t *unicastPayload() { return (uint8_t *)(clearData()+sizeof(UnicastHeaderSt)); }
};

typedef std::function<void(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi)> UnicastReceiveRadioPacketHandler;

struct UnicastBindedPort_st {
    UnicastReceiveRadioPacketHandler handler;
    uint16_t port;
};
typedef UnicastBindedPort_st UnicastBindedPort_t;

class Unicast {
public:
	Unicast(PacketBuf *pbuf) { packetbuf = pbuf; packetbuf->setUnicast(this); }
    void setup(void) {}
    void loop(void);
public:
    uint8_t send(UnicastPacket *pkt, uint32_t target, bool initHeader);
    uint8_t send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port);
    void receiveRadioPacket(uint8_t *p, uint16_t size, uint32_t f, int16_t  r);
    bool isPortAvailable(uint16_t port) const;
    bool bindPort(uint16_t port, UnicastReceiveRadioPacketHandler h);
private:
    static void radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt);
    void radioPacketSent(uint8_t status, RadioPacket *pkt);
private:
    PacketBuf *packetbuf;
    std::list<UnicastBindedPort_t> mBindedPorts;
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
private: // For multipath
    uint32_t *mRepeaters{nullptr};
    uint8_t mRepeatersSize{0};
};

} // namespace espmeshmesh
