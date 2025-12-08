#ifdef ESPMESH_STARPATH_ENABLED
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
}

StarPathPacket::StarPathPacket(PacketBufProtocol * owner, PacketType pktType, uint16_t port, uint16_t dataSize, SentStatusHandler cb): RadioPacket(owner, cb) {
    allocClearData(dataSize);
    starPathHeader()->pktType = pktType;
    starPathHeader()->port = port;
}

void StarPathPacket::allocClearData(uint16_t size) {
    uint16_t totalSize = size + sizeof(StarPathHeaderSt);
    RadioPacket::allocClearData(totalSize);
    memset(clearData(), 0x00, totalSize);
    starPathHeader()->protocol = MeshAddress::SRC_STARPATH;
    starPathHeader()->payloadLength = size;
}

uint8_t *StarPathPacket::starPathPayload() {
    return (uint8_t *)(clearData()+sizeof(StarPathHeaderSt));
}

StarPathProtocol::StarPathProtocol(bool isCoordinator, PacketBuf *pbuf, ReceiveHandler rx_fn): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_STARPATH), mRecvDups() {
    LIB_LOGD(TAG, "StarPathProtocol constructor isCoordinator %d", isCoordinator);
    if(isCoordinator) {
        mNodeState = Associated;
        mCoordinatorId = mPacketBuf->nodeId();
    }
    mPacketBuf->setRecvHandler(MeshAddress::SRC_STARPATH,  this);
#if LIB_LOG_LEVEL >= LIB_LOG_LEVEL_VERY_VERBOSE
    mRecvDups.setDebug(true);
#endif
}

void StarPathProtocol::setup() {
    if(mNodeState == Free) {
        // First beacon
        mNextDiscoveryBeaconDeadline = millis() + 1500 + (random_uint32() % 1000);
    }
}

void StarPathProtocol::loop() {
    mRecvDups.loop();
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
 * @return True if the packet is sent, false otherwise.
 */
bool StarPathProtocol::send(const uint8_t *data, uint16_t size, MeshAddress target, SentStatusHandler handler) {
    if(mNodeState != Associated) {
        return false;
    } 

    espmeshmesh_PathRouting pathrouting = espmeshmesh_PathRouting_init_zero;
    pathrouting.source_address = mPacketBuf->nodeId();
    pathrouting.target_address = target.address == MeshAddress::coordinatorAddress ? mCoordinatorId : target.address;
    pathrouting.direction = espmeshmesh_PathDirection_TO_COORDINATOR;

    sendDataPacket(data, size, &pathrouting, target.port, handler);
    return true;
}

/**
 * Encrypts and sends a packet.
 * @param pkt The packet to send.
 * @param handler The handler to call when the packet is sent.
 */
void StarPathProtocol::sendRawPacket(StarPathPacket *pkt, uint32_t target, SentStatusHandler handler) {
    pkt->starPathHeader()->seqno = mLastSequenceNum++;
    pkt->encryptClearData();
    pkt->setCallback(handler);
    pkt->fill80211(target == MeshAddress::broadCastAddress ? nullptr : (uint8_t *)&target, mPacketBuf->nodeIdPtr());
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
        LIB_LOGE(TAG, "send not associated");
    } else {
        // Calculate the size of the message
        pb_ostream_t sizestream = {0};
        pb_encode_ex(&sizestream, espmeshmesh_PathRouting_fields, path_struct, PB_ENCODE_DELIMITED);
        
        StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, port, sizestream.bytes_written + size, handler);

        // Encode the message
        LIB_LOGD(TAG, "sendDataPacket size %d size %d", sizestream.bytes_written, size);
        pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
        pb_encode_ex(&stream, espmeshmesh_PathRouting_fields, path_struct, PB_ENCODE_DELIMITED);
        memcpy(pkt->starPathPayload() + sizestream.bytes_written, data, size);
        
        // Encrypt and send the packet
        sendRawPacket(pkt, mNeighbourId, handler);
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
        StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacket, target.port, sizestream.bytes_written, handler);

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
        sendRawPacket(pkt, mNeighbourId, handler);
    }
}

void StarPathProtocol::sendBeacon(const pb_msgdesc_t *fields, const void *src_struct, StarPathPacket::PacketType pktType) {
    // Calculate the size of the message
    pb_ostream_t sizestream = {0};
    pb_encode(&sizestream, fields, src_struct);

    // Create a new packet
    StarPathPacket *pkt = new StarPathPacket(this, pktType, 0, sizestream.bytes_written, nullptr);

    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
    pb_encode(&stream, fields, src_struct);

    // Encrypt and send the packet
    sendRawPacket(pkt, MeshAddress::broadCastAddress, nullptr);
}

void StarPathProtocol::radioPacketRecv(uint8_t *payload, uint16_t size, uint32_t from, int16_t rssi) {
    //LIB_LOGV(TAG, "radioPacketRecv from %06X rssi %d size %d", from, rssi, size);
    if(size < sizeof(StarPathHeader)) {
        LIB_LOGE(TAG, "radioPacketRecv invalid size %d but required at least %d", size, sizeof(StarPathHeader));
        return;
    }
  
    StarPathPacket *pkt = new StarPathPacket(this, nullptr);
    pkt->fromRawData(payload, size);

    StarPathHeader *header = pkt->starPathHeader();

    if (mRecvDups.checkDuplicateTable(from, header->port, header->seqno)) {
        LIB_LOGV(TAG, "duplicated packet received from %06X:%02X with seq %d", from, header->port, header->seqno);
        return;
    }

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
                StarPathPacket *newpkt = new StarPathPacket(this, nullptr);
                // Copy the packet data
                newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
                // Increment the number of retransmissions
                newpkt->starPathHeader()->flags++;
                // Send the packet again
                sendRawPacket(newpkt, mNeighbourId, oldpkt->getCallback());
            } else {
                // We have reached the maximum number of retransmissions, we send a NACK packet to the source
                LIB_LOGE(TAG, "transmission error for %06X from %06X after %d try", pkt->target8211(), oldpkt->sourceAddress(), header->flags & STARPATH_FLAG_RETRANSMIT_MASK);
                // disassociate from the neighbour
                if(pkt->target8211() == mNeighbourId) {
                    disassociateFromNeighbour();
                }
                // Send a NACK packet to the source
                if(oldpkt->sourceAddress() != MeshAddress::noAddress) {
                    sendDataPacketNackPacket(oldpkt->sourceAddress());
                }
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

/**
 * If the path already contains my node id then it is a loop.
 */
bool StarPathProtocol::containsLoop(const uint32_t *repeaters, uint8_t repeaters_count) const {
    for(uint8_t i = 0; i < repeaters_count; i++) {
        if(repeaters[i] == mPacketBuf->nodeId()) {
            return true;
        }
    }
    return false;
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

#define TOTAL_COST_MARGIN 20

/**
 * Handles a notification beacon received from a neighbour.
 * Notificaion beacons are sent from neighbours when they are associated or not associated to the coordinator. 
 * When the neighbour is associated to the coordinator, we can use this to find the best path to the coordinator.
 * When our neighbour is not associated to the coordinator anymore, we can disassociate from him.
 */
void StarPathProtocol::handleNotificationBeacon(StarPathPacket *pkt, uint32_t from, int16_t rssi) {
    if(mNodeState == Associated && !iAmCoordinator()) {
        LIB_LOGD(TAG, "handleNotificationBeacon from %06X", from);
        // If I am associated, I can handle the notification beacon

        // Decode the notification beacon
        espmeshmesh_NotificationBeacon notificationbeacon;
        pb_istream_t istream = pb_istream_from_buffer(pkt->starPathPayload(), pkt->starPathHeader()->payloadLength);
        if(!pb_decode(&istream, espmeshmesh_NotificationBeacon_fields, &notificationbeacon)) {
            LIB_LOGE(TAG, "handleNotificationBeacon decode notification beacon failed %s", PB_GET_ERROR(&istream));
            return;
        }

        if(mCoordinatorId == notificationbeacon.target_address) {
            // We share the same coordinator
            if(mNeighbourId == from) {
                // Sender of packet is my neighboor
                if(notificationbeacon.target_address == MeshAddress::noAddress) {
                    // My neighboor has lost association with the coordinator, I have to disassociate from him
                    disassociateFromNeighbour();
                }
            } else {
                // Sender of packet is not my neighboor. 
                if(notificationbeacon.target_address != MeshAddress::noAddress) {
                    // Sender packet is associated to the coordinator. Check if the path is better than the current path

                    //Calculate the full cost of the path rssi and hops
                    uint16_t fullCost = calculateFullCost(rssi, notificationbeacon.repeaters_count) + TOTAL_COST_MARGIN;
                    LIB_LOGD(TAG, "handleNotificationBeacon fullCost %d < %d", fullCost, mNeighbourCost);
                    fullCost += calculateTestbedCosts(mPacketBuf->nodeId(), from);
                    // Check if the path cost is better than the current path cost
                    if(fullCost < mNeighbourCost) {
                        // Check if the path contains a loop
                        if(!containsLoop(notificationbeacon.repeaters, notificationbeacon.repeaters_count)) {
                            // Associate to the new neighbour
                            LIB_LOGD(TAG, "handleNotificationBeacon associate to new neighbour %06X cost %d hops %d", from, fullCost, notificationbeacon.repeaters_count);
                            associateToNeighbour(from, notificationbeacon.target_address, fullCost, notificationbeacon.repeaters_count);
                        }
                    }
                }
            }
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

        // If the path contains a loop, we discard the beacon reply
        if(containsLoop(discoverybeaconreply.repeaters, discoverybeaconreply.repeaters_count)) {
            LIB_LOGE(TAG, "handleDiscoveryBeaconReply loop found in neighbour %02X repeaters path", from);
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
        espmeshmesh_PathRouting pathrouting = espmeshmesh_PathRouting_init_zero;
        pb_istream_t istream = pb_istream_from_buffer(payload, header->payloadLength);
        if(!pb_decode_ex(&istream, espmeshmesh_PathRouting_fields, &pathrouting, PB_DECODE_DELIMITED)) {
            LIB_LOGE(TAG, "handleDataPacket decode path routing failed %s", PB_GET_ERROR(&istream));
        }

        if(pathrouting.direction == espmeshmesh_PathDirection_TO_COORDINATOR) {
            // We are going to the coordinator

            // Get real payload of the packet, we need it later
            uint8_t *realPayload = payload + (header->payloadLength - istream.bytes_left);
            uint16_t realPayloadLength = istream.bytes_left;

            if(pathrouting.target_address == mPacketBuf->nodeId()) {
                // We are the target, we can handle the data packet

                // The packet is for me, make a mesh address from the path routing structure
                LIB_LOGD(TAG, "handleDataPacket to coordinator last hop");
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
                    handleDataPresentationPacket(realPayload+1, realPayloadLength-1, sourceAddress, &pathrouting, rssi);
                } else {
                    this->callReceiveHandler(realPayload, realPayloadLength, sourceAddress, rssi);
                }

            } else {
                // We are not the target, we can forward the data packet to the coordinator
                // We add ourselves to the path routing structure

                if(pathrouting.repeaters_count >= STARPATH_MAX_PATH_LENGTH - 1) {
                    LIB_LOGE(TAG, "handleDataPacket too many hops. Discarding packet.");
                    return;
                }

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
        LIB_LOGE(TAG, "handleDataPacket not associated. Discarding packet.");
        sendDataPacketNackPacket(from);
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
    nodepresentationrx.has_node_presentation = true;
    strncpy(nodepresentationrx.node_presentation.hostname, nodepresentation.hostname, 48);
    strncpy(nodepresentationrx.node_presentation.firmware_version, nodepresentation.firmware_version, 16);
    strncpy(nodepresentationrx.node_presentation.compile_time, nodepresentation.compile_time, 48);
    strncpy(nodepresentationrx.node_presentation.lib_version, nodepresentation.lib_version, 16);
    nodepresentationrx.has_path_routing = true; 
    nodepresentationrx.path_routing.source_address = pathrouting->source_address;
    nodepresentationrx.path_routing.target_address = pathrouting->target_address;
    nodepresentationrx.path_routing.repeaters_count = pathrouting->repeaters_count;
    nodepresentationrx.path_routing.rssi_count = pathrouting->rssi_count + 1;
    nodepresentationrx.path_routing.hop_index = pathrouting->hop_index;

    for(uint8_t i = 0; i < pathrouting->hop_index; i++) {
        nodepresentationrx.path_routing.repeaters[i] = pathrouting->repeaters[i];
        nodepresentationrx.path_routing.rssi[i] = pathrouting->rssi[i];
    }

    // Add the RSSI of the last hop
    nodepresentationrx.path_routing.rssi[nodepresentationrx.path_routing.rssi_count-1] = rssi;

    pb_ostream_t sizestream = {0};
    if(!pb_encode(&sizestream, espmeshmesh_NodePresentationRx_fields, &nodepresentationrx)) {
        LIB_LOGE(TAG, "handleDataPresentationPacket encode error %s", PB_GET_ERROR(&sizestream));
        return;
    }

    LIB_LOGD(TAG, "handleDataPresentationPacket size %d", sizestream.bytes_written);
    uint8_t *encoded = new uint8_t[sizestream.bytes_written + 1];

    uint8_t msgid = espmeshmesh_NodePresentationRx_msgid;
    pb_ostream_t ostream = pb_ostream_from_buffer(encoded, sizestream.bytes_written+1);
    pb_write(&ostream, &msgid, 1);
    pb_encode(&ostream, espmeshmesh_NodePresentationRx_fields, &nodepresentationrx);
    this->callReceiveHandler(encoded, ostream.bytes_written, from, rssi);
    delete[] encoded;

}

/**
 * Sends a discovery beacon packet using broadcast.
 */
void StarPathProtocol::sendDiscoveryBeacon() {
    LIB_LOGD(TAG, "sendDiscoveryBeacon");
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeacon, 0, 0, nullptr);
    sendRawPacket(pkt, MeshAddress::broadCastAddress, nullptr);
}

void StarPathProtocol::sendNotificationBeacon() {
    LIB_LOGD(TAG, "sendNotificationBeacon");
    
    // This beacon is sent to all neighbours, it notify my state. If not associated, the target address is 0.
    espmeshmesh_NotificationBeacon notificationbeacon = espmeshmesh_NotificationBeacon_init_zero;
    notificationbeacon.target_address = mCoordinatorId;
    notificationbeacon.total_cost = mNeighbourCost;
    notificationbeacon.repeaters_count = mRepeaters.size();
    for(uint8_t i = 0; i < mRepeaters.size(); i++) {
        notificationbeacon.repeaters[i] = mRepeaters[i];
    }

    sendBeacon(espmeshmesh_NotificationBeacon_fields, &notificationbeacon, StarPathPacket::NotificationBeacon);
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
    discoverybeaconreply.repeaters_count = mRepeaters.size();
    for(uint8_t i = 0; i < mRepeaters.size(); i++) {
        discoverybeaconreply.repeaters[i] = mRepeaters[i];
    }

    // Calculate the size of the message
    pb_ostream_t sizestream = {0};
    pb_encode(&sizestream, espmeshmesh_DiscoveryBeaconReply_fields, &discoverybeaconreply);

    // Create a new packet
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DiscoveryBeaconReply, 0, sizestream.bytes_written, nullptr);
    LIB_LOGD(TAG, "sendDiscoveryBeaconReply size %d", sizestream.bytes_written);

    // Encode the message
    pb_ostream_t stream = pb_ostream_from_buffer(pkt->starPathPayload(), sizestream.bytes_written);
    pb_encode(&stream, espmeshmesh_DiscoveryBeaconReply_fields, &discoverybeaconreply);

    // Encrypt and send the packet
    sendRawPacket(pkt, target, nullptr);
}

/**
 * Sends a data packet nack packet to the data packet originator.
 */
void StarPathProtocol::sendDataPacketNackPacket(uint32_t target) {
    LIB_LOGD(TAG, "sendDataPacketNackPacket to %06X", target);
    StarPathPacket *pkt = new StarPathPacket(this, StarPathPacket::DataPacketNack, 0, 0, nullptr);
    // Encrypt and send the packet
    sendRawPacket(pkt, target, nullptr);
}

/**
 * Sends a data presentation packet to the coordinator.
 */
void StarPathProtocol::sendPresentationPacket() {
    LIB_LOGD(TAG, "sendPresentationPacket");
    espmeshmesh_NodePresentation nodepresentation = espmeshmesh_NodePresentation_init_zero;
    strncpy(nodepresentation.hostname, espmeshmesh::EspMeshMesh::getInstance()->hostname().c_str(), 48);
    strncpy(nodepresentation.firmware_version, espmeshmesh::EspMeshMesh::getInstance()->fwVersion().c_str(), 16);
    strncpy(nodepresentation.compile_time, espmeshmesh::EspMeshMesh::getInstance()->compileTime().c_str(), 48);
    strncpy(nodepresentation.lib_version, espmeshmesh::EspMeshMesh::getInstance()->libVersion().c_str(), 16);
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

    // Sanity check. We can't associate to ourselves.
    if(iAmCoordinator() || neighbourId == mPacketBuf->nodeId()) {
        LIB_LOGE(TAG, "associateToNeighbour neighbourId %06X is myself or coordinator", neighbourId);
        return;
    }

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

    // Sanity check. We can't disassociate from ourselves.
    if(iAmCoordinator()) {
        // If I am the coordinator, I can't disassociate from myself
        LIB_LOGE(TAG, "disassociateFromNeighbour I am the coordinator");
        return;
    }

    mNodeState = Free;
    mNeighbourId = MeshAddress::noAddress;
    mCoordinatorId = MeshAddress::noAddress;
    mCoordinatorHops = 0;
    mNeighbourCost = UINT16_MAX;
    mRepeaters.clear();
    // Notify neighbours that I am not associated anymore
    mNextNotificationBeaconDeadline = millis() + 500 + (random_uint32() % 100);
    // Resume sending association beacons
    mNextDiscoveryBeaconDeadline = millis() + 1000 + (random_uint32() % 100);
}

}
#endif