#pragma once
#include "packetbuf.h"
#include "recvdups.h"

#include <pb.h>

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
struct StarPathBindedPort_st {
    ReceiveHandler handler;
    uint16_t port;
};
typedef StarPathBindedPort_st StarPathBindedPort_t;

class StarPathPacket: public RadioPacket {
public:
    enum PacketType { DiscoveryBeacon, DiscoveryBeaconReply, DataPacket, DataPacketNack, NotificationBeacon };
    explicit StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb = nullptr);
    explicit StarPathPacket(PacketBufProtocol * owner, PacketType pktType, SentStatusHandler cb = nullptr);
    void allocClearData(uint16_t size) override;
public:
	StarPathHeader *starPathHeader() { return (StarPathHeader *)clearData(); }
	uint8_t *starPathPayload();
private:
    PacketType mPktType;
};

class StarPathProtocol: public PacketBufProtocol {
    struct Neighbour_st {
        uint32_t id;
        uint32_t coordinatorId;
        uint16_t cost;
        uint8_t hops;
    } __attribute__ ((packed));
    typedef struct Neighbour_st Neighbour_t;

    enum NodeState {Free, Binded, Associated};
public:
    StarPathProtocol(bool isCoordinator, PacketBuf *pbuf, ReceiveHandler rx_fn = nullptr);
    void setup() override;
    void loop() override;
public:
    void send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler = nullptr);
private:
    void sendRawPacket(StarPathPacket *pkt, SentStatusHandler handler = nullptr);
    void sendDataPacket(const uint8_t *data, uint16_t size, const void *path_struct, uint8_t port, SentStatusHandler handler = nullptr);
    void sendDataPacket(const pb_msgdesc_t *fields, const void *src_struct, uint8_t cmdid, MeshAddress target, SentStatusHandler handler = nullptr);
    void sendBeacon(const pb_msgdesc_t *fields, const void *src_struct, SentStatusHandler handler = nullptr);
public:
    void radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) override;
    void radioPacketSent(uint8_t status, RadioPacket *pkt) override;
private:
    uint16_t calculateCost(int16_t rssi) const;
    uint16_t calculateFullCost(int16_t rssi, uint8_t hops) const;
    int16_t calculateTestbedCosts(uint32_t source, uint32_t target) const;
private:
    void handleDiscoveryBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi);
    void handleNotificationBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi);
    void handleDiscoveryBeaconReply(StarPathPacket *pkt, uint32_t from);
    void handleDataPacket(StarPathPacket *pkt, uint32_t from, int16_t rssi);
    void handleDataPacketNack(StarPathPacket *pkt, uint32_t from);
    void handleDataPresentationPacket(uint8_t *payload, uint16_t length, const MeshAddress &from, const void *pathroutingptr, int16_t rssi);
private:
    void sendDiscoveryBeacon();
    void sendNotificationBeacon();
    void sendDiscoveryBeaconReply(uint32_t target, int16_t rssi);
    void sendDataPacketNackPacket(uint32_t target);
    void sendPresentationPacket();
private:
    void analyseBeacons();
    void associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint16_t neighbourCost, uint8_t coordinatorHops);
    void disassociateFromNeighbour();
private:
    uint16_t mLastSequenceNum = 0;
    RecvDups mRecvDups;
    NodeState mNodeState{Free};
    uint32_t mNeighbourId{0};
    uint16_t mNeighbourCost{UINT16_MAX};
    uint32_t mCoordinatorId{0};
    uint8_t mCoordinatorHops{0};
    std::vector<uint32_t> mRepeaters;
private:
    uint32_t mNextDiscoveryBeaconDeadline{0};
    uint32_t mNextNotificationBeaconDeadline{0};
    uint32_t mBeaconAnalysisDeadline{0};
    uint32_t mBeaconReplyDeadline{0};
    uint32_t mBeaconReplyTarget{0};
    int16_t mBeaconReplyRssi{0};
    std::vector<Neighbour_t> mNeighbours;
};

}

