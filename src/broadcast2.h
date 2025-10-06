#pragma once
#include "defines.h"
#include "packetbuf.h"
#include <functional>

namespace espmeshmesh {

struct broadcast2_header_st {
	uint8_t protocol;
	uint8_t flags;
	uint8_t port;
	uint16_t lenght;
} __attribute__ ((packed));


typedef struct broadcast2_header_st broadcast2_header_t;

class BroadCast2Packet: public RadioPacket {
public:
	explicit BroadCast2Packet(PacketBufProtocol * owner, SentStatusHandler cb): RadioPacket(owner, cb) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast2_header_t *broadcastHeader() { return (broadcast2_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast2_header_st); }
};

class Broadcast2: public PacketBufProtocol {
public:
	Broadcast2(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr): PacketBufProtocol(pbuf, rx_fn, SRC_BROADCAST2){}

	uint8_t send(const uint8_t *data, uint16_t size, uint16_t port, SentStatusHandler handler = nullptr);
	void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;

};

} // namespace espmeshmesh
