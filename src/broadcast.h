#pragma once
#include "defines.h"
#include "packetbuf.h"

namespace espmeshmesh {

struct broadcast_header_st {
	uint8_t protocol;
	uint16_t lenght;
} __attribute__ ((packed));

typedef struct broadcast_header_st broadcast_header_t;

class BroadCastPacket: public RadioPacket {
public:
	explicit BroadCastPacket(PacketBufProtocol * owner, SentStatusHandler cb): RadioPacket(owner, cb) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast_header_t *broadcastHeader() { return (broadcast_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast_header_st); }
};


class Broadcast: public PacketBufProtocol {
public:
	Broadcast(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr): PacketBufProtocol(pbuf, rx_fn, SRC_BROADCAST){}
	uint8_t send(const uint8_t *data, uint16_t size);
	void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;

};

} // namespace espmeshmesh
