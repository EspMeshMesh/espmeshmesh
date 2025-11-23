#include "starpath.h"
#include "defines.h"
#include "commands.h"
#include "log.h"
#include <functional>
#include <cstring>

static const char *TAG = "espmeshmesh.starpath";

namespace espmeshmesh {

struct StarPathDiscoveryBeaconReply_st {
    uint32_t coordinatorId;
    int16_t rssi;
    uint8_t hops;
} __attribute__ ((packed));
typedef struct StarPathDiscoveryBeaconReply_st StarPathDiscoveryBeaconReply;

struct StarPathPresentationData_st {
    uint8_t command;
    uint32_t sourceAddress;
    uint32_t targetAddress;
} __attribute__ ((packed));
typedef struct StarPathPresentationData_st StarPathPresentationData;


struct StarPathPresentationRequest_st {
    uint8_t command;
    uint32_t sourceAddress;
    uint32_t targetAddress;
    uint32_t repeaters[STARPATH_MAX_PATH_LENGTH];
    int16_t rssi[STARPATH_MAX_PATH_LENGTH];
    uint8_t hopsCount;
} __attribute__ ((packed));
typedef struct StarPathPresentationRequest_st StarPathPresentationRequest;

StarPathPacket::StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb): RadioPacket(owner, cb) {
    mPktType = DataPacket;
}

StarPathPacket::StarPathPacket(PacketBufProtocol * owner, PacketType pktType, SentStatusHandler cb): RadioPacket(owner, cb) {
    mPktType = pktType;
}

void StarPathPacket::allocClearData(uint16_t size) {
    RadioPacket::allocClearData(size + sizeof(StarPathHeaderSt) + (mPktType == DataPacket ? sizeof(StarPathPath) : 0));
    starPathHeader()->payloadLength = size;
    starPathHeader()->pktType = mPktType;
}

uint8_t *StarPathPacket::starPathPayload() {
    if(starPathHeader()->pktType == DataPacket) {
        return (uint8_t *)(clearData()+sizeof(StarPathHeaderSt) + sizeof(StarPathPath));
    }
    return (uint8_t *)(clearData()+sizeof(StarPathHeaderSt));
}

StarPathProtocol::StarPathProtocol(bool isCoordinator, PacketBuf *pbuf, ReceiveHandler rx_fn): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_STARPATH) {
    LIB_LOGD(TAG, "StarPathProtocol constructor isCoordinator %d", isCoordinator);
    if(isCoordinator) {
        mNodeState = Associated;
        mCoordinatorId = mPacketBuf->nodeId();
    }
    mPacketBuf->setRecvHandler(MeshAddress::SRC_STARPATH,  this);
}

void StarPathProtocol::setup() {
    if(mNodeState == Free) {
        // First beacon
        mNextBeaconDeadline = millis() + 1500 + (random_uint32() % 1000);
    }
}

void StarPathProtocol::loop() {
    uint32_t now = millis();

    if(mNodeState == Free) {
        if(mNextBeaconDeadline != 0 && now > mNextBeaconDeadline) {
            mNextBeaconDeadline = now + 1000 + (random_uint32() % 64);
            mBeaconAnalysisDeadline = now + 900;
            sendDiscoveryBeacon();
        }

        if(mBeaconAnalysisDeadline != 0 && now > mBeaconAnalysisDeadline) {
            mBeaconAnalysisDeadline = 0;
            analyseBeacons();
        }
    }

    if(mNodeState == Associated) {
        if(mBeaconReplyDeadline != 0 && now > mBeaconReplyDeadline) {
            mBeaconReplyDeadline = 0;
            sendDiscoveryBeaconReply(mBeaconReplyTarget, mBeaconReplyRssi);
            mBeaconReplyTarget = 0;
            mBeaconReplyRssi = 0;
        }
    }
}

uint8_t StarPathProtocol::send(StarPathPacket *pkt, bool initHeader, SentStatusHandler handler) {
    return PKT_SEND_ERR;
}

uint8_t StarPathProtocol::send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        LIB_LOGE(TAG, "StarPath send not associated");
        return PKT_SEND_ERR;
    }

    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, handler);
    pkt->allocClearData(size);
    pkt->starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    pkt->starPathHeader()->port = target.port;
    pkt->starPathHeader()->seqno = ++mLastSequenceNum;

    StarPathPath *path = pkt->starPathPath();
    path->direction = ToCoordinator;
    path->hopIndex = 0;
    path->sourceAddress = mPacketBuf->nodeId();
    path->targetAddress = target.address;
    path->hopsCount = mCoordinatorHops;

    if(size > 0) memcpy(pkt->starPathPayload(), data, size);
    
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&mNeighbourId, mPacketBuf->nodeIdPtr());
    return mPacketBuf->send(pkt);
}

void StarPathProtocol::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
    LIB_LOGV(TAG, "StarPath radioPacketRecv from %06X rssi %d size %d", from, rssi, size);
    if(size < sizeof(StarPathHeader)) {
        LIB_LOGE(TAG, "StarPath radioPacketRecv invalid size %d but required at least %d", size, sizeof(StarPathHeader));
        return;
    }

    StarPathPacket *pkt = new StarPathPacket(this, nullptr);
    pkt->fromRawData(payload, size);

    StarPathHeader *header = pkt->starPathHeader();
    LIB_LOGV(TAG, "StarPath radioPacketRecv pktType %d port %d", header->pktType, header->port);
    const uint8_t *data = pkt->starPathPayload();
    if(header->pktType == StarPathPacket::DataPacket) {
        // If the packet is a data packet, I can handle it
        bool pktUsed = handleDataPacket(pkt, from, rssi);
        // If the packet is used, I can't delete it
        if(pktUsed) pkt = nullptr;
    } else if(header->pktType == StarPathPacket::DiscoveryBeacon) {
        handleDiscoveryBeacon(pkt, from, rssi);
    } else if(header->pktType == StarPathPacket::DiscoveryBeaconReply) {
        handleDiscoveryBeaconReply(pkt, from);
    }
    else {
        // TODO: Handle other packet types
    }

    if(pkt) delete pkt;
}

void StarPathProtocol::radioPacketSent(uint8_t status, RadioPacket *pkt) {

}

/**
 * Returns the deadline for sending a discovery beacon reply.
 * The reply is sent every 50ms + a random delay up to 64ms.
 */
uint32_t StarPathProtocol::calculateBeaconReplyDeadline() const {
    return millis() + 50 + (random_uint32() % 64);
}

/**
 * Sends a discovery beacon using broadcast.
 */
uint8_t StarPathProtocol::sendDiscoveryBeacon() {
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeacon, nullptr);
    pkt->allocClearData(0);
    pkt->starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    pkt->starPathHeader()->port = 0;
    pkt->encryptClearData();

    LIB_LOGD(TAG, "StarPath sendDiscoveryBeacon");
    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());

    uint8_t res = mPacketBuf->send(pkt);
    if (res == PKT_SEND_ERR) {
        delete pkt;
    }
    return res;
}

void StarPathProtocol::handleDiscoveryBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    LIB_LOGD(TAG, "StarPath handleDiscoveryBeacon");
    if(mNodeState == Associated && mBeaconReplyTarget == 0) {
        // If I am associated and I'm not replying to another beacon, I can reply to the beacon
        mBeaconReplyTarget = from;
        mBeaconReplyRssi = rssi;
        mBeaconReplyDeadline = calculateBeaconReplyDeadline();
    }
}

uint8_t StarPathProtocol::sendDiscoveryBeaconReply(uint32_t target, int16_t rssi) {
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeaconReply, nullptr);
    pkt->allocClearData(sizeof(StarPathDiscoveryBeaconReply));
    pkt->starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    pkt->starPathHeader()->port = 0;

    StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)pkt->starPathPayload();
    reply->coordinatorId = mCoordinatorId;
    reply->hops = mCoordinatorHops;
    reply->rssi = rssi;

    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    return mPacketBuf->send(pkt);
}

void StarPathProtocol::handleDiscoveryBeaconReply(StarPathPacket *pkt, uint32_t from) {
    LIB_LOGD(TAG, "StarPath handleDiscoveryBeaconReply from %06X", from);
    if(mNodeState == Free) {
        // If I am free, I can handle the a beacon reply
        if(pkt->starPathHeader()->payloadLength != sizeof(StarPathDiscoveryBeaconReply)) {
            LIB_LOGE(TAG, "StarPath handleDiscoveryBeaconReply invalid data size %d but required %d", pkt->starPathHeader()->payloadLength, sizeof(StarPathDiscoveryBeaconReply));
            return;
        }

        StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)pkt->starPathPayload();
        mNeighbours.push_back({from, reply->coordinatorId, reply->rssi, reply->hops});
        LIB_LOGD(TAG, "StarPath handleDiscoveryBeaconReply from %06X hops %d rssi %d", from, reply->hops, reply->rssi);
    }
}

void StarPathProtocol::analyseBeacons() {
    LIB_LOGD(TAG, "StarPath analyseBeacons");
    uint32_t bestNeighbourId = 0;
    uint32_t bestCoordinatorId = 0;
    int16_t bestNeighbourCost = INT16_MAX;
    uint8_t bestNeighbourHops = 0;

    for(auto &neighbour : mNeighbours) {
        LIB_LOGD(TAG, "StarPath analyseBeacon from %06X hops %d rssi %d", neighbour.id, neighbour.hops, neighbour.rssi);
        int16_t cost = -neighbour.rssi;
        if(cost < bestNeighbourCost) { 
            bestNeighbourCost = cost;
            bestNeighbourId = neighbour.id;
            bestCoordinatorId = neighbour.coordinatorId;
            bestNeighbourHops = neighbour.hops;
        }
    }
    if(bestNeighbourId != 0) {
        associateToNeighbour(bestNeighbourId, bestCoordinatorId, bestNeighbourHops);
    }
    mNeighbours.clear();
}

void StarPathProtocol::associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint8_t coordinatorHops) {
    LIB_LOGD(TAG, "StarPath associateToNeighbour");
    mNodeState = Associated;
    mNeighbourId = neighbourId;
    mCoordinatorId = coordinatorId;
    mCoordinatorHops = coordinatorHops;
    // Stop sending association beacons
    mNextBeaconDeadline = 0;
    sendPresentationPacket();
}

bool StarPathProtocol::handleDataPacket(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    LIB_LOGD(TAG, "StarPath handleDataPacket");
    StarPathHeader *header = pkt->starPathHeader();
    StarPathPath *path = pkt->starPathPath();

    if(mNodeState == Associated) {
        path->routerAddressses[path->hopIndex] = mPacketBuf->nodeId();
        path->hopsRssi[path->hopIndex] = rssi;
        path->hopIndex++;
        // If I am associated, I can handle the data packet
        if(path->direction == ToCoordinator) {
            // If the path is to the coordinator, I can handle the data packet
            LIB_LOGD(TAG, "StarPath handleDataPacket to coordinator");

            if(path->targetAddress == mPacketBuf->nodeId()) {
                // The packet is for me
                LIB_LOGD(TAG, "StarPath handleDataPacket to coordinator last hop");
                MeshAddress sourceAddress = MeshAddress(
                    header->port, 
                    path->sourceAddress, 
                    (const uint8_t *)(path->routerAddressses+0), 
                    path->hopIndex, 
                    true
                );
				sourceAddress.sourceProtocol = MeshAddress::SRC_STARPATH;
                if(pkt->starPathHeader()->port == 0 && pkt->starPathHeader()->payloadLength > 0 && pkt->starPathPayload()[0] == CMD_NODE_PRESENTATION_REP) {
                    // Handle the special case of a presentation packet
                    handleDataPresentationPacket(pkt, sourceAddress);
                } else {
                    this->callReceiveHandler(pkt->starPathPayload(), header->payloadLength, sourceAddress, rssi);
                }
            } else {
                // Is not for me, I can forward the data packet
                LIB_LOGD(TAG, "StarPath handleDataPacket to coordinator not last hop");
			    uint8_t res = send(pkt, false, nullptr);
                if(res == PKT_SEND_ERR) {
                    LIB_LOGE(TAG, "StarPath handleDataPacket to coordinator not last hop send error");
                }
                return true;
            }
        } else {
            // If the path is to a node, I can forward the data packet
            LIB_LOGD(TAG, "StarPath handleDataPacket to node");
        }
    }
    return false;
}

void StarPathProtocol::handleDataPresentationPacket(StarPathPacket *pkt, const MeshAddress &from) {
    LIB_LOGD(TAG, "StarPath handleDataPresentationPacket hops %d", pkt->starPathPath()->hopIndex);
    StarPathPresentationRequest data;
    data.command = CMD_NODE_PRESENTATION_REP;
    data.sourceAddress = from.address;
    data.targetAddress = mCoordinatorId;
    for(uint8_t i = 0; i < pkt->starPathPath()->hopIndex; i++) {
        data.repeaters[i] = pkt->starPathPath()->routerAddressses[i];
        data.rssi[i] = pkt->starPathPath()->hopsRssi[i];
    }
    data.hopsCount = pkt->starPathPath()->hopIndex;
    this->callReceiveHandler((uint8_t *)&data, sizeof(StarPathPresentationRequest), from, 0);
}

uint8_t StarPathProtocol::sendPresentationPacket() {
    LIB_LOGD(TAG, "StarPath sendPresentationPacket");
    StarPathPresentationData data;
    data.command = CMD_NODE_PRESENTATION_REP;
    data.sourceAddress = mPacketBuf->nodeId();
    data.targetAddress = mCoordinatorId;
    MeshAddress target = MeshAddress(0, mCoordinatorId);
    target.sourceProtocol = MeshAddress::SRC_STARPATH;
    return send((uint8_t *)&data, sizeof(StarPathPresentationData), target, nullptr);
}

}