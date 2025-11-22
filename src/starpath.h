#pragma once
#include "packetbuf.h"
#include "recvdups.h"

namespace espmeshmesh {

struct StarPathHeaderSt {
    uint8_t protocol;
    uint8_t flags;
    uint8_t port;
    uint8_t pktType;
    uint16_t seqno;
    uint16_t length;
} __attribute__ ((packed));
typedef struct StarPathHeaderSt StarPathHeader;

struct StarPathBindedPort_st {
    ReceiveHandler handler;
    uint16_t port;
};
typedef StarPathBindedPort_st StarPathBindedPort_t;

class StarPathPacket: public RadioPacket {
public:
    enum PacketType { DiscoveryBeacon, DiscoveryBeaconReply };
    explicit StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr): RadioPacket(owner, cb) {}
public:
    void allocClearData(uint16_t size) override;
public:
	StarPathHeader *startPathHeader() { return (StarPathHeader *)clearData(); }
	uint8_t *starPathPayload() { return (uint8_t *)(clearData()+sizeof(StarPathHeaderSt)); }
};

class StarPathProtocol: public PacketBufProtocol {
    struct Neighbour_st {
        uint32_t id;
        uint32_t coordinatorId;
        int16_t rssi;
        uint8_t hops;
    } __attribute__ ((packed));
    typedef struct Neighbour_st Neighbour_t;

    enum NodeState {Free, Binded, Associated};
public:
    StarPathProtocol(bool isCoordinator, PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr);
    void setup() override;
    void loop() override;
public:
    uint8_t send(StarPathPacket *pkt, bool initHeader, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    uint32_t calculateBeaconReplyDeadline() const;
    uint8_t sendDiscoveryBeacon();
    void handleDiscoveryBeacon(StarPathHeader *header, const uint8_t *data, uint16_t dataSize, uint32_t from, int16_t rssi);
    uint8_t sendDiscoveryBeaconReply(uint32_t target, int16_t rssi);
    void handleDiscoveryBeaconReply(StarPathHeader *header, const uint8_t *data, uint16_t dataSize, uint32_t from);
    void analyseBeacons();
    void associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint8_t coordinatorHops);
private:
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
    NodeState mNodeState{Free};
    uint32_t mNeighbourId{0};
    uint32_t mCoordinatorId{0};
    uint8_t mCoordinatorHops{0};
private:
    uint32_t mNextBeaconDeadline{0};
    uint32_t mBeaconAnalysisDeadline{0};
    uint32_t mBeaconReplyDeadline{0};
    uint32_t mBeaconReplyTarget{0};
    int16_t mBeaconReplyRssi{0};
    std::vector<Neighbour_t> mNeighbours;
};

}

