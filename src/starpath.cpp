#include "starpath.h"
#include "defines.h"
#include "commands.h"
#include "espmeshmesh.h"
#include "log.h"
#include <functional>
#include <cstring>

#include <pb_encode.h>
#include <pb_decode.h>
#include "protoc/notificationbeacon.pb.h"
#include "protoc/disoverybeaconreply.pb.h"
#include "protoc/nodepresentation.pb.h"
#include "protoc/pathrouting.pb.h"
#include "protoc/nodepresentationrx.pb.h"

static const char *TAG = "espmeshmesh.starpath";

#define STARPATH_FLAG_RETRANSMIT_MASK 0x0F
#define STARPATH_MAX_RETRANSMISSIONS 0x04

namespace espmeshmesh {

StarPathPacket::StarPathPacket(PacketBufProtocol * owner, SentStatusHandler cb): RadioPacket(owner, cb) {
    mPktType = DataPacket;
}

StarPathPacket::StarPathPacket(PacketBufProtocol * owner, PacketType pktType, SentStatusHandler cb): RadioPacket(owner, cb) {
    mPktType = pktType;
}

void StarPathPacket::allocClearData(uint16_t size) {
    uint16_t totalSize = size + sizeof(StarPathHeaderSt);
    RadioPacket::allocClearData(totalSize);
    memset(clearData(), 0x00, totalSize);
    starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    starPathHeader()->payloadLength = size;
    starPathHeader()->pktType = mPktType;
}

uint8_t *StarPathPacket::starPathPayload() {
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
        mNextDiscoveryBeaconDeadline = millis() + 1500 + (random_uint32() % 1000);
    }
}

void StarPathProtocol::loop() {
    uint32_t now = millis();

    if(mNodeState == Free) {
        if(mNextDiscoveryBeaconDeadline != 0 && now > mNextDiscoveryBeaconDeadline) {
            mNextDiscoveryBeaconDeadline = now + 1000 + (random_uint32() % 64);
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

        if(mNextNotificationBeaconDeadline != 0 && now > mNextNotificationBeaconDeadline) {
            mNextNotificationBeaconDeadline = 0;
            sendNotificationBeacon();
        }
    }
}

/**
 * Sends a raw payload to a target address.
 * @param data The buffer with the data to send.
 * @param size The size of the data to send.
 * @param target The target address to send the packet to.
 * @param handler The handler to call when the packet is sent.
 */
void StarPathProtocol::send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        LIB_LOGE(TAG, "StarPath send not associated");
    } else {
        espmeshmesh_PathRouting pathrouting = espmeshmesh_PathRouting_init_zero;
        pathrouting.source_address = mPacketBuf->nodeId();
        pathrouting.target_address = target.address;
        pathrouting.direction = espmeshmesh_PathDirection_TO_COORDINATOR;

        sendDataPacket(data, size, &pathrouting, target.port, handler);
    }
}

/**
 * Encrypts and sends a packet.
 * @param pkt The packet to send.
 * @param handler The handler to call when the packet is sent.
 */
void StarPathProtocol::sendRawPacket(StarPathPacket *pkt, SentStatusHandler handler) {
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&mNeighbourId, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

/**
 * Sends a raw payload with a path routing structure already defined.
 * @param data The buffer with the data to send.
 * @param size The size of the data to send.
 * @param path_struct The path routing structure to send.
 * @param target The target address to send the packet to.
 * @param handler The handler to call when the packet is sent.
 */
void StarPathProtocol::sendDataPacket(const uint8_t *data, uint16_t size, const void *path_struct, uint8_t port, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        LIB_LOGE(TAG, "StarPath send not associated");
    } else {
        // Calculate the size of the message
        pb_ostream_t sizestream = {0};
        pb_encode_ex(&sizestream, espmeshmesh_PathRouting_fields, path_struct, PB_ENCODE_DELIMITED);
        
        // Create a new packet
        StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, handler);
        pkt->allocClearData(sizestream.bytes_written + size);        
        pkt->starPathHeader()->port = port;
        pkt->starPathHeader()->seqno = ++mLastSequenceNum;

        // Encode the message
        pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
        pb_encode_ex(&sizestream, espmeshmesh_PathRouting_fields, path_struct, PB_ENCODE_DELIMITED);
        memcpy(pkt->starPathPayload() + sizestream.bytes_written, data, size);

        // Encrypt and send the packet
        sendRawPacket(pkt, handler);
    }
}

/**
 * Sends a packet with a path routing structure  and datausing Protobuf.
 * @param fields The fields structure of the message to send.
 * @param src_struct The source of data structure to send.
 * @param target The target address to send the packet to.
 * @param handler The handler to call when the packet is sent.
 */
void StarPathProtocol::sendDataPacket(const pb_msgdesc_t *fields, const void *src_struct, uint8_t cmdid, MeshAddress target, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        LIB_LOGE(TAG, "StarPath send not associated");
    } else {

        // Create a new path routing structure
        espmeshmesh_PathRouting path = espmeshmesh_PathRouting_init_zero;
        path.source_address = mPacketBuf->nodeId();
        path.target_address = target.address;
        path.direction = espmeshmesh_PathDirection_TO_COORDINATOR;

        // Calculate the size of the message
        pb_ostream_t sizestream = {0};
        pb_encode_ex(&sizestream, espmeshmesh_PathRouting_fields, &path, PB_ENCODE_DELIMITED);
        pb_write(&sizestream, &cmdid, 1);
        pb_encode(&sizestream, fields, src_struct);
        
        // Create a new packet
        StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, handler);
        pkt->allocClearData(sizestream.bytes_written);        
        pkt->starPathHeader()->port = target.port;
        pkt->starPathHeader()->seqno = ++mLastSequenceNum;

        // Encode the message with framing
        pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
        if(!pb_encode_ex(&stream, espmeshmesh_PathRouting_fields, &path, PB_ENCODE_DELIMITED)) {
            LIB_LOGE(TAG, "send encode error on %s", PB_GET_ERROR(&stream));
            delete pkt;
            return;
        }
        pb_write(&stream, &cmdid, 1);
        if(!pb_encode(&stream, fields, src_struct)) {
            LIB_LOGE(TAG, "send encode error on %s", PB_GET_ERROR(&stream));
            delete pkt;
            return;
        }

        // Encrypt and send the packet
        sendRawPacket(pkt, handler);
    }
}

void StarPathProtocol::sendBeacon(const pb_msgdesc_t *fields, const void *src_struct, SentStatusHandler handler) {
    // Calculate the size of the message
    pb_ostream_t sizestream = {0};
    pb_encode(&sizestream, fields, src_struct);

    // Create a new packet
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::NotificationBeacon, nullptr);

    // Allocate the packet
    pkt->allocClearData(sizestream.bytes_written);
    pkt->starPathHeader()->port = 0;

    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
    pb_encode(&stream, fields, src_struct);

    // Encrypt and send the packet
    sendRawPacket(pkt, handler);
}

void StarPathProtocol::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
    LIB_LOGV(TAG, "radioPacketRecv from %06X rssi %d size %d", from, rssi, size);
    if(size < sizeof(StarPathHeader)) {
        LIB_LOGE(TAG, "radioPacketRecv invalid size %d but required at least %d", size, sizeof(StarPathHeader));
        return;
    }

    StarPathPacket *pkt = new StarPathPacket(this, nullptr);
    pkt->fromRawData(payload, size);

    StarPathHeader *header = pkt->starPathHeader();
    if(header->pktType == StarPathPacket::DataPacket) {
        handleDataPacket(pkt, from, rssi);
    } else if(header->pktType == StarPathPacket::DataPacketNack) {
        handleDataPacketNack(pkt, from);
    } else if(header->pktType == StarPathPacket::DiscoveryBeacon) {
        handleDiscoveryBeacon(pkt, from, rssi);
    } else if(header->pktType == StarPathPacket::DiscoveryBeaconReply) {
        handleDiscoveryBeaconReply(pkt, from);
    } else if(header->pktType == StarPathPacket::NotificationBeacon) {
        handleNotificationBeacon(pkt, from, rssi);
    } else {
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
                newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
                newpkt->starPathHeader()->flags++;
                sendRawPacket(newpkt, nullptr);
            } else {
                LIB_LOGE(TAG, "transmission error for %06X after %d try", pkt->target8211(), header->flags & STARPATH_FLAG_RETRANSMIT_MASK);
                sendDataPacketNackPacket(pkt->target8211());
            }
        }
    }
}

uint16_t StarPathProtocol::calculateCost(int16_t rssi) const {
    const int16_t esp32RssiMax = 0;
    const int16_t esp32RssiMin = -80;

    if(rssi < esp32RssiMin) rssi = esp32RssiMin;

    int16_t cost = 100 - (rssi*100 - esp32RssiMin*100) / (esp32RssiMax-esp32RssiMin);
    return cost;
}

uint16_t StarPathProtocol::calculateFullCost(int16_t rssi, uint8_t hops) const {
    return calculateCost(rssi) + hops*30;
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

void StarPathProtocol::handleDiscoveryBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    LIB_LOGD(TAG, "handleDiscoveryBeacon");
    if(mNodeState == Associated && mBeaconReplyTarget == 0) {
        // If I am associated and I'm not replying to another beacon, I can reply to the beacon
        mBeaconReplyTarget = from;
        mBeaconReplyRssi = rssi;
        mBeaconReplyDeadline = millis() + 50 + (random_uint32() % 64);
    }
}

/**
 * Handles a notification beacon received from a neighbour.
 * Notificaion beacons are sent from neighbours. We can use this to find the best path to the coordinator.
 * Notification beacons are handled only if we are associated.
 */
void StarPathProtocol::handleNotificationBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    LIB_LOGD(TAG, "handleNotificationBeacon from %06X", from);
    if(mNodeState == Associated) {
        // If I am associated, I can handle the notification beacon

        // Decode the notification beacon
        espmeshmesh_NotificationBeacon notificationbeacon;
        pb_istream_t istream = pb_istream_from_buffer(pkt->starPathPayload(), pkt->starPathHeader()->payloadLength);
        if(!pb_decode(&istream, espmeshmesh_NotificationBeacon_fields, &notificationbeacon)) {
            LIB_LOGE(TAG, "handleNotificationBeacon decode notification beacon failed %s", PB_GET_ERROR(&istream));
            return;
        }

        // If we share the same coordinator and the neighbour is not the current neighbour
        if(mCoordinatorId == notificationbeacon.target_address && mNeighbourId != from) {
            // Calculate the full cost of the path rssi and hops
            uint16_t fullCost = calculateFullCost(rssi, notificationbeacon.repeaters_count);
            // Check if the path cost is better than the current path cost
            if(fullCost < mNeighbourCost) {
                // Check if the path contains a loop
                bool loop_found = false;
                for(int i = 0; i < notificationbeacon.repeaters_count; i++) {
                    if(notificationbeacon.repeaters[i] == mPacketBuf->nodeId()) {
                        loop_found = true;
                        break;
                    }
                }
                if(!loop_found) {
                    // Associate to the new neighbour
                    LIB_LOGD(TAG, "handleNotificationBeacon associate to new neighbour %06X cost %d hops %d", from, fullCost, notificationbeacon.repeaters_count);
                    //associateToNeighbour(from, notificationbeacon.target_address, fullCost, notificationbeacon.repeaters_count);
                }
            }
        }

        // Out neighbour notify that is not associated anymore
        if(notificationbeacon.target_address == 0 && mNeighbourId == from) {
            LIB_LOGD(TAG, "handleNotificationBeacon disassociate from neighbour %06X", from);
            //disassociateFromNeighbour();
        }
    }
}

void StarPathProtocol::handleDiscoveryBeaconReply(StarPathPacket *pkt, uint32_t from) {
    LIB_LOGD(TAG, "handleDiscoveryBeaconReply from %06X", from);
    if(mNodeState == Free) {
        // If I am free, I can handle the a beacon reply

        // Decode the discovery beacon reply
        espmeshmesh_DiscoveryBeaconReply discoverybeaconreply;
        pb_istream_t istream = pb_istream_from_buffer(pkt->starPathPayload(), pkt->starPathHeader()->payloadLength);
        if(!pb_decode(&istream, espmeshmesh_DiscoveryBeaconReply_fields, &discoverybeaconreply)) {
            LIB_LOGE(TAG, "handleDiscoveryBeaconReply decode discovery beacon reply failed %s", PB_GET_ERROR(&istream));
            return;
        }

        // Add the neighbour to the discovery list
        mNeighbours.push_back({
            from, discoverybeaconreply.target_address, 
            calculateFullCost(discoverybeaconreply.incoming_rssi, (uint8_t)discoverybeaconreply.hops), 
            (uint8_t)discoverybeaconreply.hops
        });
    }
}

void StarPathProtocol::handleDataPacket(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    StarPathHeader *header = pkt->starPathHeader();
    uint8_t *payload = pkt->starPathPayload();
    if(mNodeState == Associated) {
        // If I am associated, I can handle the data packet

        // Decode the path routing structure
        espmeshmesh_PathRouting pathrouting;
        pb_istream_t istream = pb_istream_from_buffer(payload, header->payloadLength);
        if(!pb_decode_ex(&istream, espmeshmesh_PathRouting_fields, &pathrouting, PB_DECODE_DELIMITED)) {
            LIB_LOGE(TAG, "handleDataPacket2 decode path routing failed %s", PB_GET_ERROR(&istream));
        }

        if(pathrouting.direction == espmeshmesh_PathDirection_TO_COORDINATOR) {
            // We are going to the coordinator

            // Get real payload of the packet, we need it later
            LIB_LOGD(TAG, "handleDataPacket2 payload length %d path length %d", header->payloadLength, header->payloadLength - istream.bytes_left);
            uint8_t *realPayload = payload + (header->payloadLength - istream.bytes_left);
            uint16_t realPayloadLength = istream.bytes_left;

            LIB_LOGD(TAG, "handleDataPacket2 %02X %02X %02X %02X %02X %02X %02X %02X", realPayload[0], realPayload[1], realPayload[2], realPayload[3], realPayload[4], realPayload[5], realPayload[6], realPayload[7]);

            if(pathrouting.target_address == mPacketBuf->nodeId()) {
                // We are the target, we can handle the data packet

                // The packet is for me, make a mesh address from the path routing structure
                LIB_LOGD(TAG, "handleDataPacket2 to coordinator last hop");
                MeshAddress sourceAddress = MeshAddress(
                    header->port, 
                    pathrouting.source_address, 
                    (const uint8_t *)(pathrouting.repeaters+0), 
                    pathrouting.hop_index, 
                    true
                );
                sourceAddress.sourceProtocol = MeshAddress::SRC_STARPATH;
                
                // Call the receive handler
                if(realPayload[0] == PROTO_NODE_PRESENTATION_REP) {
                    handleDataPresentationPacket(realPayload, realPayloadLength, sourceAddress, &pathrouting, rssi);
                } else {
                    this->callReceiveHandler(realPayload, realPayloadLength, sourceAddress, rssi);
                }

            } else {
                // We are not the target, we can forward the data packet to the coordinator
                
                // We add ourselves to the path routing structure
                pathrouting.repeaters[pathrouting.repeaters_count++] = mPacketBuf->nodeId();
                pathrouting.rssi[pathrouting.rssi_count++] = rssi;
                pathrouting.hop_index++;
    
                // Forward the data packet to the next router
                sendDataPacket(realPayload, realPayloadLength, &pathrouting, header->port, nullptr);    
            }
            
        } else {
            // We are going to a node
        }
    } else {
        LIB_LOGE(TAG, "handleDataPacket2 not associated");
    }
}

void StarPathProtocol::handleDataPacketNack(StarPathPacket *pkt, uint32_t from) {
    LIB_LOGD(TAG, "handleDataPacketNack");
    if(from == mNeighbourId) disassociateFromNeighbour();
}

/**
 * When we receive a presentation packet, we need to forward the presentation request packet to the coordinator.
 */
void StarPathProtocol::handleDataPresentationPacket(uint8_t *payload, uint16_t length, const MeshAddress &from, const void *pathroutingptr, int16_t rssi) {
    espmeshmesh_PathRouting *pathrouting = (espmeshmesh_PathRouting *)pathroutingptr;
    LIB_LOGD(TAG, "handleDataPresentationPacket hops %d", pathrouting->hop_index);

    espmeshmesh_NodePresentation nodepresentation = espmeshmesh_NodePresentation_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(payload, length);
    if(!pb_decode(&istream, espmeshmesh_NodePresentation_fields, &nodepresentation)) {
        LIB_LOGE(TAG, "handleDataPresentationPacket decode node presentation failed %s", PB_GET_ERROR(&istream));
        return;
    }

    espmeshmesh_NodePresentationRx nodepresentationrx = espmeshmesh_NodePresentationRx_init_zero;
    strncpy(nodepresentationrx.node_presentation.hostname, nodepresentation.hostname, 48);
    strncpy(nodepresentationrx.node_presentation.firmware_version, nodepresentation.firmware_version, 48);
    strncpy(nodepresentationrx.node_presentation.compile_time, nodepresentation.compile_time, 48);
    nodepresentationrx.path_routing.source_address = pathrouting->source_address;
    nodepresentationrx.path_routing.target_address = pathrouting->target_address;
    nodepresentationrx.path_routing.repeaters_count = pathrouting->repeaters_count;
    nodepresentationrx.path_routing.rssi_count = pathrouting->rssi_count;
    nodepresentationrx.path_routing.hop_index = pathrouting->hop_index;

    for(uint8_t i = 0; i < pathrouting->hop_index; i++) {
        nodepresentationrx.path_routing.repeaters[i] = pathrouting->repeaters[i];
        nodepresentationrx.path_routing.rssi[i] = pathrouting->rssi[i];
    }

    pb_ostream_t sizestream = {0};
    pb_encode(&sizestream, espmeshmesh_NodePresentationRx_fields, &nodepresentationrx);
    uint8_t *encoded = new uint8_t[sizestream.bytes_written];

    pb_ostream_t ostream = pb_ostream_from_buffer(encoded, sizestream.bytes_written);
    pb_encode(&ostream, espmeshmesh_NodePresentationRx_fields, &nodepresentationrx);
    send(encoded, sizestream.bytes_written, from, nullptr);
    delete[] encoded;

    this->callReceiveHandler(encoded, sizestream.bytes_written, from, rssi);
}

/**
 * Sends a discovery beacon packet using broadcast.
 */
void StarPathProtocol::sendDiscoveryBeacon() {
    LIB_LOGD(TAG, "sendDiscoveryBeacon");
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeacon, nullptr);
    pkt->allocClearData(0);
    pkt->starPathHeader()->port = 0;
    pkt->encryptClearData();

    pkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

void StarPathProtocol::sendNotificationBeacon() {
    if(mNodeState == Associated) {
        LIB_LOGD(TAG, "sendNotificationBeacon");
        // Fill output message
        espmeshmesh_NotificationBeacon notificationbeacon = espmeshmesh_NotificationBeacon_init_zero;
        notificationbeacon.target_address = mCoordinatorId;
        notificationbeacon.total_cost = mNeighbourCost;
        notificationbeacon.repeaters_count = mRepeaters.size();
        for(uint8_t i = 0; i < mRepeaters.size(); i++) {
            notificationbeacon.repeaters[i] = mRepeaters[i];
        }

        sendBeacon(espmeshmesh_NotificationBeacon_fields, &notificationbeacon, nullptr);
    } else {
        LIB_LOGW(TAG, "sendNotificationBeacon not associated");
    }
}

/**
 * Sends a discovery beacon reply packet to the becaon originator.
 */
void StarPathProtocol::sendDiscoveryBeaconReply(uint32_t target, int16_t rssi) {
    LIB_LOGD(TAG, "sendDiscoveryBeaconReply to %06X rssi %d", target, rssi);
    // Fill output message
    espmeshmesh_DiscoveryBeaconReply discoverybeaconreply;
    discoverybeaconreply.target_address = mCoordinatorId;
    discoverybeaconreply.incoming_rssi = rssi;
    discoverybeaconreply.hops = mCoordinatorHops;

    // Calculate the size of the message
    pb_ostream_t sizestream = {0};
    pb_encode(&sizestream, espmeshmesh_DiscoveryBeaconReply_fields, &discoverybeaconreply);

    // Create a new packet
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeaconReply, nullptr);
    LIB_LOGD(TAG, "sendDiscoveryBeaconReply size %d", sizestream.bytes_written);
    pkt->allocClearData(sizestream.bytes_written);
    pkt->starPathHeader()->port = 0;

    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
    pb_encode(&stream, espmeshmesh_DiscoveryBeaconReply_fields, &discoverybeaconreply);

    // Encrypt and send the packet
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

/**
 * Sends a data packet nack packet to the data packet originator.
 */
void StarPathProtocol::sendDataPacketNackPacket(uint32_t target) {
    LIB_LOGD(TAG, "sendDataPacketNackPacket to %06X", target);
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacketNack, nullptr);
    pkt->allocClearData(0);
    pkt->starPathHeader()->port = 0;
    pkt->encryptClearData();
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
    mPacketBuf->send(pkt);
}

/**
 * Sends a data presentation packet to the coordinator.
 */
void StarPathProtocol::sendPresentationPacket() {
    LIB_LOGD(TAG, "sendPresentationPacket2");
    espmeshmesh_NodePresentation nodepresentation;
    strncpy(nodepresentation.hostname, espmeshmesh::EspMeshMesh::getInstance()->hostname().c_str(), 48);
    strncpy(nodepresentation.firmware_version, espmeshmesh::EspMeshMesh::getInstance()->fwVersion().c_str(), 48);
    strncpy(nodepresentation.compile_time, espmeshmesh::EspMeshMesh::getInstance()->compileTime().c_str(), 48);
    MeshAddress target = MeshAddress(0, mCoordinatorId);
    target.sourceProtocol = MeshAddress::SRC_STARPATH;
    sendDataPacket(espmeshmesh_NodePresentation_fields, &nodepresentation, espmeshmesh_NodePresentation_msgid, target, nullptr);
}

void StarPathProtocol::analyseBeacons() {
    LIB_LOGD(TAG, "analyseBeacons");
    uint32_t bestNeighbourId = 0;
    uint32_t bestCoordinatorId = 0;
    int16_t bestNeighbourCost = INT16_MAX;
    uint8_t bestNeighbourHops = 0;

    for(auto &neighbour : mNeighbours) {
        int16_t totalCost = neighbour.cost;
        LIB_LOGD(TAG, "analyseBeacon from %06X hops %d cost %d totalCost %d", neighbour.id, neighbour.hops, neighbour.cost, totalCost);
        totalCost += calculateTestbedCosts(mPacketBuf->nodeId(), neighbour.id);
        if(totalCost < bestNeighbourCost) { 
            bestNeighbourCost = totalCost;
            bestNeighbourId = neighbour.id;
            bestCoordinatorId = neighbour.coordinatorId;
            bestNeighbourHops = neighbour.hops;
        }
    }
    if(bestNeighbourId != 0) {
        LIB_LOGD(TAG, "analyseBeacons best neighbour %06X cost %d hops %d", bestNeighbourId, bestNeighbourCost, bestNeighbourHops);
        associateToNeighbour(bestNeighbourId, bestCoordinatorId, bestNeighbourCost, bestNeighbourHops);
    }
    mNeighbours.clear();
}

void StarPathProtocol::associateToNeighbour(uint32_t neighbourId, uint32_t coordinatorId, uint16_t neighbourCost, uint8_t coordinatorHops) {
    LIB_LOGD(TAG, "associateToNeighbour");
    mNodeState = Associated;
    mNeighbourId = neighbourId;
    mCoordinatorId = coordinatorId;
    mCoordinatorHops = coordinatorHops + 1;
    mNeighbourCost = neighbourCost;
    // Stop sending association beacons
    mNextDiscoveryBeaconDeadline = 0;
    // Notify neighbours that I am associated with a new neighbour
    mNextNotificationBeaconDeadline = millis() + 500 + (random_uint32() % 100);
    // Send a presentation packet to the coordinator
    sendPresentationPacket();
}

void StarPathProtocol::disassociateFromNeighbour() {
    LIB_LOGD(TAG, "disassociateFromNeighbour");
    mNodeState = Free;
    mNeighbourId = 0;
    mCoordinatorId = 0;
    mCoordinatorHops = 0;
    mNeighbourCost = UINT16_MAX;
    mRepeaters.clear();
    // Notify neighbours that I am not associated anymore
    mNextNotificationBeaconDeadline = millis() + 500 + (random_uint32() % 100);
    // Resume sending association beacons
    mNextDiscoveryBeaconDeadline = millis() + 1000 + (random_uint32() % 100);
}

}