#pragma once
#include "packetbuf.h"
#include "recvdups.h"

namespace espmeshmesh {

#define STARPATH_MAX_PATH_LENGTH 16

struct StarPathHeaderSt {
    uint8_t protocol;
    uint8_t flags;
    uint8_t port;
    uint8_t pktType;
    uint16_t seqno;
    uint16_t payloadLength;
} __attribute__ ((packed));
typedef struct StarPathHeaderSt StarPathHeader;

enum StarPathDirection {
    ToCoordinator,
    ToNode
};

struct StarPathPath_st {
    uint32_t sourceAddress;
    uint32_t targetAddress;
    uint32_t routerAddressses[16];
    int16_t hopsRssi[16];
    uint8_t hopsCount;
    uint8_t hopIndex;
    StarPathDirection direction;
} __attribute__ ((packed));
typedef struct StarPathPath_st StarPathPath;

struct StarPathBindedPort_st {
    ReceiveHandler handler;
    uint16_t port;
};
typedef StarPathBindedPort_st StarPathBindedPort_t;

class StarPathPacket: public RadioPacket {
public:
    enum PacketType { DiscoveryBeacon, DiscoveryBeaconReply, DataPacket, DataPacketNack };
    explicit StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr);
    explicit StarPathPacket(PacketBufProtocol * owner, PacketType pktType, SentStatusHandler cb = nullptr);
    void allocClearData(uint16_t size) override;
public:
	StarPathHeader *starPathHeader() { return (StarPathHeader *)clearData(); }
	StarPathPath *starPathPath() { return (StarPathPath *)(clearData()+sizeof(StarPathHeaderSt)); }
	uint8_t *starPathPayload();
private:
    PacketType mPktType;
};

class StarPathProtocol: public PacketBufProtocol {
    struct Neighbour_st {
        uint32_t id;
        uint32_t coordinatorId;
        int16_t cost;
        uint8_t hops;
    } __attribute__ ((packed));
    typedef struct Neighbour_st Neighbour_t;

    enum NodeState {Free, Binded, Associated};
public:
    StarPathProtocol(bool isCoordinator, PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr);
    void setup() override;
    void loop() override;
public:
    uint8_t send(StarPathPacket *pkt, SentStatusHandler handler = nullptr);
    uint8_t send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler = nullptr);
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    int16_t calculateCost(int16_t rssi) const;
    int16_t calculateTestbedCosts(uint32_t source, uint32_t target) const;
    uint32_t calculateBeaconReplyDeadline() const;
    void handleDiscoveryBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi);
    void handleDiscoveryBeaconReply(StarPathPacket *pkt, uint32_t from);
    void analyseBeacons();
    void associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint8_t coordinatorHops);
    void disassociateFromNeighbour();
    bool handleDataPacket(StarPathPacket *pkt, uint32_t from, int16_t rssi);
    void handleDataPacketNack(StarPathPacket *pkt, uint32_t from);
    void handleDataPresentationPacket(StarPathPacket *pkt, const MeshAddress &from);
    uint8_t sendDiscoveryBeacon();
    uint8_t sendDiscoveryBeaconReply(uint32_t target, int16_t rssi);
    uint8_t sendDataPacketNackPacket(uint32_t target);
    uint8_t sendPresentationPacket();
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

