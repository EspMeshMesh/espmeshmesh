#include "broadcast2.h"
#include "log.h"
#include <cstring>

static const char *TAG = "espmeshmesh.broadcast2";

namespace espmeshmesh {

void BroadCast2Packet::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(broadcast2_header_st));
}

Broadcast2::Broadcast2(PacketBuf *pbuf) {
	mPacketbuf = pbuf;
	mPacketbuf->setRecvHandler(PROTOCOL_BROADCAST_V2, std::bind(&Broadcast2::recv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

uint8_t Broadcast2::send(const uint8_t *data, uint16_t size, uint8_t port, Broadcast2SentStatusHandler handler) {
	LIB_LOGV(TAG, "Broadcast2::send port %d size %d", port, size);
	BroadCast2Packet *pkt = new BroadCast2Packet(nullptr, nullptr);
	pkt->allocClearData(size);
	pkt->broadcastHeader()->protocol = PROTOCOL_BROADCAST_V2;
	pkt->broadcastHeader()->lenght = size;
	pkt->broadcastHeader()->flags = 0;
	pkt->broadcastHeader()->port = port;
	memcpy(pkt->broadcastPayload(), data, size);

    pkt->encryptClearData();
    pkt->fill80211(nullptr, mPacketbuf->nodeIdPtr());
	pkt->setSentStatusHandler(handler);
	pkt->setCallback(radioPacketSentCb, this);
	uint8_t res = mPacketbuf->send(pkt);
	if(res == PKT_SEND_ERR) delete pkt;
    return res;
}

bool Broadcast2::isPortAvailable(uint8_t port) const {
	for (Broadcast2BindedPort_t bindedPort : mBindedPorts) {
		if (bindedPort.port == port) {
			return false;
		}
	}
	return true;
}

bool Broadcast2::bindPort(uint8_t port, Broadcast2ReceiveRadioPacketHandler h) {
	if(!isPortAvailable(port)) {
		LIB_LOGE(TAG, "Broadcast2::bindPort port %d already binded", port);
		return false;
	}
	LIB_LOGV(TAG, "Broadcast2::bindPort port %d", port);
	Broadcast2BindedPort_t newhandler = {h, port};
	mBindedPorts.push_back(newhandler);
	return true;
}

void Broadcast2::unbindPort(uint8_t port) {
	LIB_LOGV(TAG, "Broadcast2::unbindPort ports size %d", mBindedPorts.size());
	for(std::list<Broadcast2BindedPort_t>::iterator it = mBindedPorts.begin(); it != mBindedPorts.end(); it++) {
		if (it->port == port) {
			mBindedPorts.erase(it);
			return;
		}
	}

	LIB_LOGE(TAG, "Broadcast2::unbindPort port %d is not binded", port);
}

void Broadcast2::recv(uint8_t *p, uint16_t size, uint32_t from, int16_t rssi) {
	broadcast2_header_t *header = (broadcast2_header_t *)p;
	LIB_LOGV(TAG, "Broadcast2::recv port %d size %d", header->port, header->lenght);

	for (Broadcast2BindedPort_t port : mBindedPorts) {
		if (port.port == header->port) {
		  port.handler(p + sizeof(broadcast2_header_t), header->lenght, from, rssi);
		}
	}
}

void Broadcast2::radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt) {
    ((Broadcast2 *)arg)->radioPacketSent(status, pkt);
}

void Broadcast2::radioPacketSent(uint8_t status, RadioPacket *pkt) {
	BroadCast2Packet *oldpkt = (BroadCast2Packet *)pkt;
	oldpkt->notifySentStatusHandler(!status);
}

}  // namespace espmeshmesh
