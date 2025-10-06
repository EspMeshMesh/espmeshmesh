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


/**
 * @brief Unicast packet
 * This class mantains the sent status of the current sent unicast packet
 * The packet will be deleted automatically when the packet is sent
 */
class UnicastPacket: public RadioPacket {
public:
	explicit UnicastPacket(PacketBufProtocol *owner, SentStatusHandler cb = nullptr): RadioPacket(owner, cb) {}
public:
    virtual void allocClearData(uint16_t size);
public:
	UnicastHeader *unicastHeader() { return (UnicastHeader *)clearData(); }
	uint8_t *unicastPayload() { return (uint8_t *)(clearData()+sizeof(UnicastHeaderSt)); }
};


class Unicast: public PacketBufProtocol {
public:
	Unicast(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr): PacketBufProtocol(pbuf, rx_fn, SRC_UNICAST), mRecvDups() {}
    void loop(void) override;
public:
    uint8_t send(UnicastPacket *pkt, uint32_t target, bool initHeader, SentStatusHandler handler = nullptr);
    uint8_t send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t  rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
};

} // namespace espmeshmesh
