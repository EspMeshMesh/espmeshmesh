#pragma once
#include "packetbuf.h"

namespace espmeshmesh {

struct broadcast_header_st {
	uint8_t protocol;
	uint16_t lenght;
} __attribute__ ((packed));

typedef struct broadcast_header_st broadcast_header_t;

/**
 * @brief Broadcast receive callback function
 * This is the callback function for the broadcast protocol. It is used to receive the broadcast packets.
 */
typedef void (*breadcast_recv_cb_fn)(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);

/**
 * @brief Broadcast protocol packet 
 * This is the packet class for the broadcast protocol. 
 * Allocate the memory for the broadcast header and the payload.
 */
class BroadCastPacket: public RadioPacket {
public:
	explicit BroadCastPacket(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast_header_t *broadcastHeader() { return (broadcast_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast_header_st); }
};

/**
 * @brief Broadcast protocol implementation
 * This is the most basic broadcast protocol implementation. It is used to send and receive broadcast 802.11 packets.
 * This implementation don't provide a port so there can be only one receive callback.
 */
class Broadcast: public PacketBufProtocol {
public:
	Broadcast(PacketBuf *pbuf);
	uint8_t send(const uint8_t *data, uint16_t size);
	void recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi);
	void setRecv_cb(breadcast_recv_cb_fn rx_fn);
private:
	PacketBuf *mPacketbuf;
	breadcast_recv_cb_fn rx_func = nullptr;
};

} // namespace espmeshmesh