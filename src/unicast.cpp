#include "unicast.h"
#include "log.h"

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.unicast";

#define UNICAST_FLAG_RETRANSMIT_MASK 0x0F
#define UNICAST_MAX_RETRANSMISSIONS 0x04

void UnicastPacket::allocClearData(uint16_t size) {
  RadioPacket::allocClearData(size + sizeof(UnicastHeaderSt));
  // Len of payload in protocol header
  unicastHeader()->lenght = size;
}

void Unicast::loop() { mRecvDups.loop(); }

uint8_t Unicast::send(UnicastPacket *pkt, uint32_t target, bool initHeader) {
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
  pkt->setCallback(radioPacketSentCb, this);
  pkt->encryptClearData();
  pkt->fill80211((uint8_t *) &target, packetbuf->nodeIdPtr());
  uint8_t res = packetbuf->send(pkt);
  if (res == PKT_SEND_ERR)
    delete pkt;
  return res;
}

uint8_t Unicast::send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port) {
  UnicastPacket *pkt = new UnicastPacket(nullptr, nullptr);
  pkt->allocClearData(size);
  pkt->unicastHeader()->port = port;
  os_memcpy(pkt->unicastPayload(), data, size);
  return send(pkt, target, true);
}

void Unicast::receiveRadioPacket(uint8_t *p, uint16_t size, uint32_t f, int16_t r) {
  UnicastHeader *header = (UnicastHeader *) p;
  LIB_LOGVV(TAG, "unicast_recv size=%d seq %d=%d", size, header->seqno, mLastSequenceNum);
  if (size < sizeof(UnicastHeaderSt) + header->lenght) {
    LIB_LOGE(TAG, "Unicast::recv invalid size %d but required %d", size, sizeof(UnicastHeaderSt) + header->lenght);
    return;
  }

  if (mRecvDups.checkDuplicateTable(f, 0, header->seqno)) {
    LIB_LOGE(TAG, "Unicast duplicated packet received from %06lX with seq %d", f, header->seqno);
    return;
  }

  for (UnicastBindedPort_t port : mBindedPorts) {
    if (port.port == header->port) {
      port.handler(port.arg, p + sizeof(UnicastHeaderSt), header->lenght, f, r);
    }
  }
}

void Unicast::bindPort(UnicastReceiveRadioPacketHandler h, void *arg, uint16_t port) {
  LIB_LOGD(TAG, "Unicast::bindPort port %d", port);
  UnicastBindedPort_t newhandler = {h, arg, port};
  mBindedPorts.push_back(newhandler);
}

void Unicast::radioPacketSentCb(void *arg, uint8_t status, RadioPacket *pkt) {
  ((Unicast *) arg)->radioPacketSent(status, pkt);
}

void Unicast::radioPacketSent(uint8_t status, RadioPacket *pkt) {
  if (status) {
    // Handle transmission error onyl with packets with clean data
    UnicastPacket *oldpkt = (UnicastPacket *) pkt;
    UnicastHeader *header = oldpkt->unicastHeader();
    if (header != nullptr) {
      if ((header->flags & UNICAST_FLAG_RETRANSMIT_MASK) < UNICAST_MAX_RETRANSMISSIONS) {
        UnicastPacket *newpkt = new UnicastPacket(radioPacketSentCb, this);
        newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
        newpkt->unicastHeader()->flags++;
        send(newpkt, pkt->target8211(), false);
      } else {
        LIB_LOGE(TAG, "Unicast::radioPacketSent transmission error for %06lX after %d try", pkt->target8211(),
                 header->flags & UNICAST_FLAG_RETRANSMIT_MASK);
        // FIXME: Signal error to packet creator
      }
    }
  }
}

}  // namespace espmeshmesh
