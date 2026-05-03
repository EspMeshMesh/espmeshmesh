#pragma once

#ifdef USE_POLITE_BROADCAST_PROTOCOL
#include "packetbuf.h"

#include <cstdint>
#include <functional>

namespace espmeshmesh {

class EspMeshMesh;

struct PoliteBroadcastHeaderSt {
    uint8_t protocol;
    uint8_t port;
	uint32_t sourceAddr;
    uint32_t destAddr;
    uint16_t payloadLenght;
    uint16_t sequenceNum;
}  __attribute__ ((packed));
typedef PoliteBroadcastHeaderSt PoliteBroadcastHeader;

#define POLITE_RANDOM_SLOT(X)   (random_uint32() % (X))
#define POLITE_DURATION_MS      105                                             // 105ms                Duration of poite broadcast procedure
#define POLITE_GUARDTIME_MS     POLITE_DURATION_MS*3                            // 105ms*3 = 315ms      Gaurdtime to avoid collisions
#define POLITE_SLOT_DURATION_MS 5                                               // 5ms                  Duration of a single tx slot
#define POLITE_SLOTS            POLITE_DURATION_MS/POLITE_SLOT_DURATION_MS      // 105ms/5ms = 21       Number of available TX slots
#define POLITE_RETX_NUM         3                                               // 3                    Retransmission of a single device
#define POLITE_RETX_SLOTS       POLITE_SLOTS/POLITE_RETX_NUM                    // 21/3 = 7             Number of available TX slots for a retransm.
#define POLITE_RECEIVED_BY      2                                               // 2                    Number of RX to terminate retrasmissions.
#define POLITE_PAST_MAX         10                                              // 10                   Number of senders to store
#define POLITE_PAST_RETAIN_TIME 60000                                           // 60 seconds            Retain time for past senders

class PolitePacket: public RadioPacket {
public:
	explicit PolitePacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr);
	explicit PolitePacket(uint8_t *data, uint16_t size);
private:
    void _setup();
public:
    virtual void allocClearData(uint16_t size);
public:
	PoliteBroadcastHeader *politeHeader() { return (PoliteBroadcastHeader *)clearData(); }
	uint8_t *politePayload() { return (uint8_t *)(clearData()+sizeof(PoliteBroadcastHeaderSt)); }
public:
    void calcDelays();
    bool checkDelay(uint32_t elapsed);
    uint8_t received() const { return mReceived; }
    bool incrementReceived();
private:
    uint8_t mDelays[POLITE_RETX_NUM];
    uint8_t mReceived{0};
};

struct PoliteBroadcastPast {
    uint32_t timestamp;
    uint32_t from;
    uint16_t sequenceNum;
};

class PoliteBroadcastProtocol: public PacketBufProtocol {
public:
    enum PoliteState { StateIdle, StateWaitEnd };
	PoliteBroadcastProtocol(PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr);
    void loop() override;
    void send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *data, uint16_t size, uint32_t fromptr, int16_t rssi);
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;

private:
    void _sendPkt(PolitePacket *pkt);
    void _sendRaw();
    void _setIdle(void);
private:
    bool _checkPastSenders(uint32_t from, uint16_t sequenceNum);
    void _addPastSender(uint32_t from, uint16_t sequenceNum);
    void _clearOlderPastSenders();
    PoliteBroadcastPast mPastSenders[POLITE_PAST_MAX];
private:
    uint32_t mTimeStamp0 = 0;
    uint32_t mTimeStamp1 = 0;
    PolitePacket *mOutPkt = nullptr;
    std::list<PolitePacket *> mOutPkts;
	uint8_t mLastSequenceNumber = 1;
    PoliteState mState = StateIdle;
};

}  // namespace espmeshmesh

#endif
