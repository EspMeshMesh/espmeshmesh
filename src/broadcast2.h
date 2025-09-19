#pragma once
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

typedef void (*breadcast_recv_cb_fn)(uint8_t *data, uint16_t size, uint8_t *from, int16_t r);

class BroadCast2Packet: public RadioPacket {
public:
	explicit BroadCast2Packet(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast2_header_t *broadcastHeader() { return (broadcast2_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast2_header_st); }
};

typedef std::function<void(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi)> Broadcast2ReceiveRadioPacketHandler;

struct Broadcast2BindedPort_st {
    Broadcast2ReceiveRadioPacketHandler handler;
    uint16_t port;
};
typedef Broadcast2BindedPort_st Broadcast2BindedPort_t;

class Broadcast2 {
public:
	Broadcast2(PacketBuf *pbuf) { packetbuf = pbuf; packetbuf->setBroadcast2(this); }
	uint8_t send(const uint8_t *data, uint16_t size, uint8_t port);
	bool isPortAvailable(uint16_t port) const;
	bool bindPort(uint16_t port, Broadcast2ReceiveRadioPacketHandler h);
	void open();
	void recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi);
private:
	PacketBuf *packetbuf;
    std::list<Broadcast2BindedPort_t> mBindedPorts;
};

} // namespace espmeshmesh