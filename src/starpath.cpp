#include "starpath.h"
#include "defines.h"
#include "log.h"
#include <functional>

static const char *TAG = "espmeshmesh.starpath";

namespace espmeshmesh {

struct StarPathDiscoveryBeaconReply_st {
    uint32_t coordinatorId;
    int16_t rssi;
    uint8_t hops;
} __attribute__ ((packed));
typedef struct StarPathDiscoveryBeaconReply_st StarPathDiscoveryBeaconReply;


void StarPathPacket::allocClearData(uint16_t size) {
    RadioPacket::allocClearData(size + sizeof(StarPathHeaderSt));
    startPathHeader()->length = size;
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
        mNextBeaconDeadline = millis() + 3000 + (random_uint32() % 1000);
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

void StarPathProtocol::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
    LIB_LOGV(TAG, "StarPath radioPacketRecv from %06X rssi %d size %d", from, rssi, size);
    if(size < sizeof(StarPathHeader)) {
        LIB_LOGE(TAG, "StarPath radioPacketRecv invalid size %d but required at least %d", size, sizeof(StarPathHeader));
        return;
    }

    StarPathHeader *header = (StarPathHeader *) payload;
    LIB_LOGV(TAG, "StarPath radioPacketRecv pktType %d port %d", header->pktType, header->port);
    if(header->pktType == StarPathPacket::DiscoveryBeacon) {
        const uint8_t *data = header->length == 0 ? nullptr : payload + sizeof(StarPathHeader);
        handleDiscoveryBeacon(header, data, header->length, from, rssi);
    }
    else if(header->pktType == StarPathPacket::DiscoveryBeaconReply) {
        const uint8_t *data = header->length == 0 ? nullptr : payload + sizeof(StarPathHeader);
        handleDiscoveryBeaconReply(header, data, header->length, from);
    }
    else {
        // TODO: Handle other packet types
    }
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
    StarPathPacket *pkt = new StarPathPacket(this);
    pkt->allocClearData(0);
    pkt->startPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    pkt->startPathHeader()->pktType = StarPathPacket::DiscoveryBeacon;
    pkt->startPathHeader()->port = 0;
    pkt->encryptClearData();

    LIB_LOGD(TAG, "StarPath sendDiscoveryBeacon");
    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());

    uint8_t res = mPacketBuf->send(pkt);
    if (res == PKT_SEND_ERR) {
        delete pkt;
    }
    return res;
}

void StarPathProtocol::handleDiscoveryBeacon(StarPathHeader *header, const uint8_t *data, uint16_t dataSize, uint32_t from, int16_t rssi) {
    LIB_LOGD(TAG, "StarPath handleDiscoveryBeacon");
    if(mNodeState == Associated && mBeaconReplyTarget == 0) {
        // If I am associated and I'm not replying to another beacon, I can reply to the beacon
        mBeaconReplyTarget = from;
        mBeaconReplyRssi = rssi;
        mBeaconReplyDeadline = calculateBeaconReplyDeadline();
    }
}

uint8_t StarPathProtocol::sendDiscoveryBeaconReply(uint32_t target, int16_t rssi) {
    StarPathPacket *pkt = new StarPathPacket(this);
    pkt->allocClearData(sizeof(StarPathDiscoveryBeaconReply));
    pkt->startPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    pkt->startPathHeader()->pktType = StarPathPacket::DiscoveryBeaconReply;
    pkt->startPathHeader()->port = 0;

    StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)pkt->starPathPayload();
    reply->coordinatorId = mCoordinatorId;
    reply->hops = mCoordinatorHops;
    reply->rssi = rssi;

    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    return mPacketBuf->send(pkt);
}

void StarPathProtocol::handleDiscoveryBeaconReply(StarPathHeader *header, const uint8_t *data, uint16_t dataSize, uint32_t from) {
    LIB_LOGD(TAG, "StarPath handleDiscoveryBeaconReply from %06X", from);
    if(mNodeState == Free) {
        // If I am free, I can handle the a beacon reply
        if(dataSize != sizeof(StarPathDiscoveryBeaconReply)) {
            LIB_LOGE(TAG, "StarPath handleDiscoveryBeaconReply invalid data size %d but required %d", dataSize, sizeof(StarPathDiscoveryBeaconReply));
            return;
        }

        StarPathDiscoveryBeaconReply *reply = (StarPathDiscoveryBeaconReply *)data;
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
}
}