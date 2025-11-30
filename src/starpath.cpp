#include "starpath.h"
#include "defines.h"
#include "commands.h"
#include "espmeshmesh.h"
#include "log.h"
#include <functional>
#include <cstring>

static const char *TAG = "espmeshmesh.starpath";

#define STARPATH_FLAG_RETRANSMIT_MASK 0x0F
#define STARPATH_MAX_RETRANSMISSIONS 0x04

namespace espmeshmesh {

struct StarPathDiscoveryBeaconReply_st {
    uint32_t coordinatorId;
    int16_t rssi;
    uint8_t hops;
} __attribute__ ((packed));
typedef struct StarPathDiscoveryBeaconReply_st StarPathDiscoveryBeaconReply;


#define STARPATH_PRESENTATION_DATA_HOSTNAME_LENGTH 16
#define STARPATH_PRESENTATION_DATA_FW_VERSION_LENGTH 16
#define STARPATH_PRESENTATION_DATA_COMPILE_TIME_LENGTH 24

/**
 * This is the paylod of the radio packet used to send a presentation data packet.
 */
struct StarPathPresentationData_st {
    uint8_t command;
    uint32_t sourceAddress;
    uint32_t targetAddress;
    char hostname[STARPATH_PRESENTATION_DATA_HOSTNAME_LENGTH];
    char fwVersion[STARPATH_PRESENTATION_DATA_FW_VERSION_LENGTH];
    char compileTime[STARPATH_PRESENTATION_DATA_COMPILE_TIME_LENGTH];
} __attribute__ ((packed));
typedef struct StarPathPresentationData_st StarPathPresentationData;

/**
 * This is the paylod of the uart packet used to forward a presentation to coordinator.
 */
struct StarPathPresentationRequest_st {
    uint8_t command;
    uint32_t sourceAddress;
    uint32_t targetAddress;
    uint32_t repeaters[STARPATH_MAX_PATH_LENGTH];
    int16_t rssi[STARPATH_MAX_PATH_LENGTH];
    uint8_t hopsCount;
    char hostname[STARPATH_PRESENTATION_DATA_HOSTNAME_LENGTH];
    char fwVersion[STARPATH_PRESENTATION_DATA_FW_VERSION_LENGTH];
    char compileTime[STARPATH_PRESENTATION_DATA_COMPILE_TIME_LENGTH];
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
    starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
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

void StarPathProtocol::send(StarPathPacket *pkt, SentStatusHandler handler) {
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&mNeighbourId, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

void StarPathProtocol::send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        LIB_LOGE(TAG, "StarPath send not associated");
    } else {
        StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, handler);
        pkt->allocClearData(size);
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
        mPacketBuf->send(pkt);    
    }
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
    } else if(header->pktType == StarPathPacket::DataPacketNack) {
        handleDataPacketNack(pkt, from);
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
    StarPathPacket *oldpkt = (StarPathPacket *)pkt;
    if(status) {
        // Handle transmission error onyl with packets with clean data
        StarPathHeader *header = oldpkt->starPathHeader();
        if(header != nullptr && header->pktType == StarPathPacket::DataPacket) {
            if((header->flags & STARPATH_FLAG_RETRANSMIT_MASK) < STARPATH_MAX_RETRANSMISSIONS) {
                StarPathPacket *newpkt = new StarPathPacket(this, StarPathPacket::DataPacket, nullptr);
                newpkt->starPathHeader()->flags++;
                newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
                send(newpkt, nullptr);
            } else {
                LIB_LOGE(TAG, "StarPath::radioPacketSent transmission error for %06X after %d try", pkt->target8211(), header->flags & STARPATH_FLAG_RETRANSMIT_MASK);
                sendDataPacketNackPacket(pkt->target8211());
            }
        }
    }
}

int16_t StarPathProtocol::calculateCost(int16_t rssi) const {
    const int16_t esp32RssiMax = 0;
    const int16_t esp32RssiMin = -80;

    if(rssi < esp32RssiMin) rssi = esp32RssiMin;

    int16_t cost = 100 - (rssi*100 - esp32RssiMin*100) / (esp32RssiMax-esp32RssiMin);
    return cost;
}

/**
 * Calculates the virtual costs toke make a path in the testbed.
 * The cost is calculated based on the source and target addresses.
 */
int16_t StarPathProtocol::calculateTestbedCosts(uint32_t source, uint32_t target) const {
    // Node4 --> Node3 ---> Node2 --> Node1 --> Coordinator
    if(source == 0xC0E5A8 && target != 0xB575B8) {
        // Node1 --> Coordinator
        return 100;
    }
    if(source == 0xB56EC4 && target != 0xC0E5A8) {
        // Node2 --> Node1
        return 100;
    }
    if(source == 0xC16BC8 && target != 0xB56EC4) {
        // Node2 --> Node1
        return 100;
    }
    return 0;
}

/**
 * Returns the deadline for sending a discovery beacon reply.
 * The reply is sent every 50ms + a random delay up to 64ms.
 */
uint32_t StarPathProtocol::calculateBeaconReplyDeadline() const {
    return millis() + 50 + (random_uint32() % 64);
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

void StarPathProtocol::handleDiscoveryBeaconReply(StarPathPacket *pkt, uint32_t from) {
    LIB_LOGD(TAG, "StarPath handleDiscoveryBeaconReply from %06X", from);
    if(mNodeState == Free) {
        // If I am free, I can handle the a beacon reply
        if(pkt->starPathHeader()->payloadLength != sizeof(StarPathDiscoveryBeaconReply)) {
            LIB_LOGE(TAG, "StarPath handleDiscoveryBeaconReply invalid data size %d but required %d", pkt->starPathHeader()->payloadLength, sizeof(StarPathDiscoveryBeaconReply));
            return;
        }

        StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)pkt->starPathPayload();
        int16_t cost = calculateCost(reply->rssi);
        mNeighbours.push_back({from, reply->coordinatorId, cost, reply->hops});
        LIB_LOGD(TAG, "StarPath handleDiscoveryBeaconReply from %06X hops %d rssi %d cost %d", from, reply->hops, reply->rssi, cost);
    }
}

void StarPathProtocol::analyseBeacons() {
    LIB_LOGD(TAG, "StarPath analyseBeacons");
    uint32_t bestNeighbourId = 0;
    uint32_t bestCoordinatorId = 0;
    int16_t bestNeighbourCost = INT16_MAX;
    uint8_t bestNeighbourHops = 0;

    for(auto &neighbour : mNeighbours) {
        int16_t totalCost = neighbour.cost + neighbour.hops*30;
        LIB_LOGD(TAG, "StarPath analyseBeacon from %06X hops %d cost %d totalCost %d", neighbour.id, neighbour.hops, neighbour.cost, totalCost);
        totalCost += calculateTestbedCosts(mPacketBuf->nodeId(), neighbour.id);
        if(totalCost < bestNeighbourCost) { 
            bestNeighbourCost = totalCost;
            bestNeighbourId = neighbour.id;
            bestCoordinatorId = neighbour.coordinatorId;
            bestNeighbourHops = neighbour.hops;
        }
    }
    if(bestNeighbourId != 0) {
        LIB_LOGD(TAG, "StarPath analyseBeacons best neighbour %06X cost %d hops %d", bestNeighbourId, bestNeighbourCost, bestNeighbourHops);
        associateToNeighbour(bestNeighbourId, bestCoordinatorId, bestNeighbourHops);
    }
    mNeighbours.clear();
}

void StarPathProtocol::associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint8_t coordinatorHops) {
    LIB_LOGD(TAG, "StarPath associateToNeighbour");
    mNodeState = Associated;
    mNeighbourId = neighbourId;
    mCoordinatorId = coordinatorId;
    mCoordinatorHops = coordinatorHops + 1;
    // Stop sending association beacons
    mNextBeaconDeadline = 0;
    sendPresentationPacket();
}

void StarPathProtocol::disassociateFromNeighbour() {
    LIB_LOGD(TAG, "StarPath disassociateFromNeighbour");
    mNodeState = Free;
    mNeighbourId = 0;
    mCoordinatorId = 0;
    mCoordinatorHops = 0;
    // Resumr sending association beacons
    mNextBeaconDeadline = millis() + 100 + (random_uint32() % 100);
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
			    send(pkt, nullptr);
                return true;
            }
        } else {
            // If the path is to a node, I can forward the data packet
            LIB_LOGD(TAG, "StarPath handleDataPacket to node");
        }
    }
    return false;
}

void StarPathProtocol::handleDataPacketNack(StarPathPacket *pkt, uint32_t from) {
    LIB_LOGD(TAG, "StarPath handleDataPacketNack");
    if(from == mNeighbourId) disassociateFromNeighbour();
}

/**
 * When we receive a presentation packet, we need to forward the presentation request packet to the coordinator.
 */
void StarPathProtocol::handleDataPresentationPacket(StarPathPacket *pkt, const MeshAddress &from) {
    LIB_LOGD(TAG, "StarPath handleDataPresentationPacket hops %d", pkt->starPathPath()->hopIndex);
    StarPathPresentationData *pktData = (StarPathPresentationData *)pkt->starPathPayload();
    StarPathPresentationRequest data;
    data.command = CMD_NODE_PRESENTATION_REP;
    data.sourceAddress = pktData->sourceAddress;
    data.targetAddress = pktData->targetAddress;
    strncpy(data.hostname, pktData->hostname, 16);
    strncpy(data.fwVersion, pktData->fwVersion, 16);
    strncpy(data.compileTime, pktData->compileTime, 24);
    for(uint8_t i = 0; i < pkt->starPathPath()->hopIndex; i++) {
        data.repeaters[i] = pkt->starPathPath()->routerAddressses[i];
        data.rssi[i] = pkt->starPathPath()->hopsRssi[i];
    }
    data.hopsCount = pkt->starPathPath()->hopIndex;
    this->callReceiveHandler((uint8_t *)&data, sizeof(StarPathPresentationRequest), from, 0);
}

/**
 * Sends a discovery beacon packet using broadcast.
 */
void StarPathProtocol::sendDiscoveryBeacon() {
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeacon, nullptr);
    pkt->allocClearData(0);
    pkt->starPathHeader()->port = 0;
    pkt->encryptClearData();

    LIB_LOGD(TAG, "StarPath sendDiscoveryBeacon");
    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());

    mPacketBuf->send(pkt);
}

/**
 * Sends a discovery beacon reply packet to the becaon originator.
 */
void StarPathProtocol::sendDiscoveryBeaconReply(uint32_t target, int16_t rssi) {
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeaconReply, nullptr);
    pkt->allocClearData(sizeof(StarPathDiscoveryBeaconReply));
    pkt->starPathHeader()->port = 0;

    StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)pkt->starPathPayload();
    reply->coordinatorId = mCoordinatorId;
    reply->hops = mCoordinatorHops;
    reply->rssi = rssi;

    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

/**
 * Sends a data packet nack packet to the data packet originator.
 */
void StarPathProtocol::sendDataPacketNackPacket(uint32_t target) {
    LIB_LOGD(TAG, "StarPath sendDataPacketNackPacket");
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacketNack, nullptr);
    pkt->allocClearData(0);
    pkt->starPathHeader()->port = 0;
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

/**
 * Sends a  data presentation packet to the coordinator.
 */
void StarPathProtocol::sendPresentationPacket() {
    LIB_LOGD(TAG, "StarPath sendPresentationPacket");
    StarPathPresentationData data;
    data.command = CMD_NODE_PRESENTATION_REP;
    data.sourceAddress = mPacketBuf->nodeId();
    data.targetAddress = mCoordinatorId;
    strncpy(data.hostname, espmeshmesh::EspMeshMesh::getInstance()->hostname().c_str(), 16);
    strncpy(data.fwVersion, espmeshmesh::EspMeshMesh::getInstance()->fwVersion().c_str(), 16);
    strncpy(data.compileTime, espmeshmesh::EspMeshMesh::getInstance()->compileTime().c_str(), 24);
    MeshAddress target = MeshAddress(0, mCoordinatorId);
    target.sourceProtocol = MeshAddress::SRC_STARPATH;
    send((uint8_t *)&data, sizeof(StarPathPresentationData), target, nullptr);
}

}