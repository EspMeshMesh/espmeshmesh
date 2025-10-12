#include "unicast.h"
#include "log.h"

#include <cstring>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.unicast";

#define UNICAST_FLAG_RETRANSMIT_MASK 0x0F
#define UNICAST_MAX_RETRANSMISSIONS 0x04

void UnicastPacket::allocClearData(uint16_t size) {
  RadioPacket::allocClearData(size + sizeof(UnicastHeaderSt));
  // Len of payload in protocol header
  unicastHeader()->lenght = size;
}

Unicast::Unicast(PacketBuf *pbuf): packetbuf(pbuf), mRecvDups() {
  packetbuf->setRecvHandler(PROTOCOL_UNICAST, std::bind(&Unicast::receiveRadioPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

void Unicast::loop() { mRecvDups.loop(); }

uint8_t Unicast::send(UnicastPacket *pkt, uint32_t target, bool initHeader, SentStatusHandler handler) {
  UnicastHeader *header = pkt->unicastHeader();
  // Fill protocol header...
  header->protocol = PROTOCOL_UNICAST;
  // Optional fields
  if (initHeader) {
    // Add flags to this packet
    header->flags = 0;
    // If is an ACK i use the last seqno
    header->seqno = ++mLastSequenceNum;
  }
  // Set this class as destination sent callback for retransmisisons
  pkt->setCallback(handler);
  pkt->encryptClearData();
  pkt->fill80211((uint8_t *) &target, mPacketBuf->nodeIdPtr());
  uint8_t res = mPacketBuf->send(pkt);
  if (res == PKT_SEND_ERR)
    delete pkt;
  return res;
}

uint8_t Unicast::send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, SentStatusHandler handler) {
  UnicastPacket *pkt = new UnicastPacket(this);
  pkt->allocClearData(size);
  pkt->unicastHeader()->port = port;
  memcpy(pkt->unicastPayload(), data, size);
  return send(pkt, target, true, handler);
}

void Unicast::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
  UnicastHeader *header = (UnicastHeader *) payload;
  LIB_LOGVV(TAG, "unicast_recv size=%d seq %d=%d", size, header->seqno, mLastSequenceNum);
  if (size < sizeof(UnicastHeaderSt) + header->lenght) {
    LIB_LOGE(TAG, "Unicast::recv invalid size %d but required %d", size, sizeof(UnicastHeaderSt) + header->lenght);
    return;
  }

  if (mRecvDups.checkDuplicateTable(from, 0, header->seqno)) {
    LIB_LOGE(TAG, "Unicast duplicated packet received from %06lX with seq %d", from, header->seqno);
    return;
  }
  this->callReceiveHandler(payload + sizeof(UnicastHeaderSt), header->lenght, from, rssi, header->port);
}

void Unicast::radioPacketSent(uint8_t status, RadioPacket *pkt) {
  UnicastPacket *oldpkt = (UnicastPacket *) pkt;

  if (status) {
    // Handle transmission error only with packets with clean data
    UnicastPacket *oldpkt = (UnicastPacket *) pkt;
    UnicastHeader *header = oldpkt->unicastHeader();
    if (header != nullptr) {
      if ((header->flags & UNICAST_FLAG_RETRANSMIT_MASK) < UNICAST_MAX_RETRANSMISSIONS) {
        UnicastPacket *newpkt = new UnicastPacket(this);
        newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
        newpkt->unicastHeader()->flags++;
        send(newpkt, pkt->target8211(), false, oldpkt->getCallback());
        return;
      } else {
        LIB_LOGE(TAG, "Unicast::radioPacketSent transmission error for %06lX after %d try", pkt->target8211(),
                 header->flags & UNICAST_FLAG_RETRANSMIT_MASK);
      }
    }
  }
  oldpkt->callCallback(status, pkt);
}

}  // namespace espmeshmesh
