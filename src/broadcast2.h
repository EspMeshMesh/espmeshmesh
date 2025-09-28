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

/**
 * @brief Broadcast2 sent status handler
 * @param status true if the packet has been sent correctly, false otherwise
 */
typedef std::function<void(bool status)> Broadcast2SentStatusHandler;

typedef struct broadcast2_header_st broadcast2_header_t;

class BroadCast2Packet: public RadioPacket {
public:
	explicit BroadCast2Packet(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) { setIsBroadcast(); }
	virtual void allocClearData(uint16_t size);
public:
	broadcast2_header_t *broadcastHeader() { return (broadcast2_header_t *)clearData(); }
	uint8_t *broadcastPayload() { return (uint8_t *)clearData()+sizeof(broadcast2_header_st); }
public:
	void setSentStatusHandler(Broadcast2SentStatusHandler handler) { mSentStatusHandler = handler; }
	Broadcast2SentStatusHandler sentStatusHandler() const { return mSentStatusHandler; }
	void notifySentStatusHandler(bool status) const { if(mSentStatusHandler) mSentStatusHandler(status); }
private:
	Broadcast2SentStatusHandler mSentStatusHandler = nullptr;
};

typedef std::function<void(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi)> Broadcast2ReceiveRadioPacketHandler;

struct Broadcast2BindedPort_st {
    Broadcast2ReceiveRadioPacketHandler handler;
    uint8_t port;
};
typedef Broadcast2BindedPort_st Broadcast2BindedPort_t;

class Broadcast2: public PacketBufProtocol {
public:
	Broadcast2(PacketBuf *pbuf);
	uint8_t send(const uint8_t *data, uint16_t size, uint8_t port, Broadcast2SentStatusHandler handler);
	bool isPortAvailable(uint8_t port) const;
	bool bindPort(uint8_t port, Broadcast2ReceiveRadioPacketHandler h);
	void unbindPort(uint8_t port);
	void recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi);
private:
	static void radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt);
	void radioPacketSent(uint8_t status, RadioPacket *pkt);
private:
	PacketBuf *mPacketbuf;
    std::list<Broadcast2BindedPort_t> mBindedPorts;
};

} // namespace espmeshmesh