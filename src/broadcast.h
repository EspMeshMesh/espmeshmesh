#pragma once
#include "packetbuf.h"
\
namespace espmeshmesh {

struct broadcast_header_st {
	uint8_t protocol;
	uint16_t lenght;
} __attribute__ ((packed));

typedef struct broadcast_header_st broadcast_header_t;

typedef void (*breadcast_recv_cb_fn)(uint8_t *data, uint16_t size, uint8_t *from, int16_t r);

class BroadCastPacket: public RadioPacket {
public:
	explicit BroadCastPacket(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast_header_t *broadcastHeader() { return (broadcast_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast_header_st); }
};


class Broadcast {
public:
	Broadcast(PacketBuf *pbuf) { packetbuf = pbuf; packetbuf->setBroadcast(this); }
	uint8_t send(const uint8_t *data, uint16_t size);
	void recv(uint8_t *p, uint16_t size, uint8_t *f, int16_t r);
	void setRecv_cb(breadcast_recv_cb_fn rx_fn);
	void open();
private:
	PacketBuf *packetbuf;
	breadcast_recv_cb_fn rx_func = nullptr;
	uint8_t seqno;
};

} // namespace espmeshmesh