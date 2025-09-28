#pragma once

#define USE_POLITE_BROADCAST_PROTOCOL
#ifdef USE_POLITE_BROADCAST_PROTOCOL

#include "packetbuf.h"
#include "defines.h"

#include <cstdint>
#include <functional>

namespace espmeshmesh {

class EspMeshMesh;

typedef std::function<void(void*, uint8_t *, uint16_t, uint32_t)> PoliteBroadcastReceiveHandler;

struct PoliteBroadcastHeaderSt {
    uint8_t protocol;
	uint32_t sourceAddr;
    uint32_t destAddr;
    uint16_t payloadLenght;
    uint16_t sequenceNum;
}  __attribute__ ((packed));
typedef PoliteBroadcastHeaderSt PoliteBroadcastHeader;

#define POLITE_RANDOM_SLOT(X)   (random_uint32() % (X))
#define POLITE_DEST_BROADCAST   0xFFFFFFFF
#define POLITE_DURATION_MS      105                                             // 105ms                Duration of poite broadcast procedure
#define POLITE_GUARDTIME_MS     POLITE_DURATION_MS*3                            // 105ms*3 = 315ms      Gaurdtime to avoid collisions
#define POLITE_SLOT_DURATION_MS 5                                               // 5ms                  Duration of a single tx slot
#define POLITE_SLOTS            POLITE_DURATION_MS/POLITE_SLOT_DURATION_MS      // 105ms/5ms = 21       Number of available TX slots
#define POLITE_RETX_NUM         3                                               // 3                    Retransmission of a single device
#define POLITE_RETX_SLOTS       POLITE_SLOTS/POLITE_RETX_NUM                    // 21/3 = 7             Number of available TX slots for a retransm.
#define POLITE_RECEIVED_BY      2                                               // 2                    Number of RX to terminate retrasmissions.

class PolitePacket: public RadioPacket {
public:
	explicit PolitePacket(pktbufSentCbFn cb, void *arg);
	explicit PolitePacket(uint8_t *data, uint16_t size);
private:
    void _setup();
public:
    virtual void allocClearData(uint16_t size);
public:
	PoliteBroadcastHeader *politeHeader() { return (PoliteBroadcastHeader *)clearData(); }
	uint8_t *politePayload() { return (uint8_t *)(clearData()+sizeof(PoliteBroadcastHeaderSt)); }
    void calcDelays();
    bool checkDelay(uint32_t elapsed);
    bool checkReceived(uint32_t source);
private:
    uint8_t mDelays[POLITE_RETX_NUM];
    uint32_t mReceived[POLITE_RECEIVED_BY];
};

class PoliteBroadcastProtocol: public PacketBufProtocol {
public:
    enum PoliteState { StateIdle, StateWaitEnd };
	PoliteBroadcastProtocol(PacketBuf *pbuf);
public:
    virtual void setup() override;
    virtual void loop() override;
public:
    void setReceivedHandler(PoliteBroadcastReceiveHandler h, void *arg);
    void receiveRadioPacket(uint8_t *data, uint16_t size, uint32_t fromptr, int16_t rssi);
    void send(const uint8_t *data, uint16_t size, uint32_t target);
private:
    static void _packetSentCb(void *arg, uint8_t status, RadioPacket *pkt);
    void _radioPacketSent(uint8_t status, PolitePacket *pkt);
    void _sendPkt(PolitePacket *pkt);
    void _sendRaw();
    void _setIdle(void);
private:
	PacketBuf *mPacketBuf;
    PoliteBroadcastReceiveHandler mReceiveHandler = nullptr;
    void* mReceiveHandlerArg = nullptr;
    uint32_t mTimeStamp0 = 0;
    uint32_t mTimeStamp1 = 0;
    PolitePacket *mOutPkt = nullptr;
    std::list<PolitePacket *> mOutPkts;
	uint8_t mLastSequenceNumber = 1;
    PoliteState mState = StateIdle;
private:
};

}  // namespace espmeshmesh

#endif
