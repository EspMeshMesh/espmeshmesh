#include "multipath.h"

#include "log.h"

#include <cstring>


namespace espmeshmesh {

static const char *TAG = "espmeshmesh.multipath";

#define MULTIPATH_FLAG_RETRANSMIT_MASK 0x0F
#define MULTIPATH_MAX_RETRANSMISSIONS 0x04

void MultiPathPacket::allocClearData(uint16_t size) {
	allocClearData(size, 0);
}

void MultiPathPacket::allocClearData(uint16_t size, uint8_t pathlen) {
	RadioPacket::allocClearData(size+sizeof(MultiPathHeaderSt)+sizeof(uint32_t)*pathlen);
	multipathHeader()->pathIndex = 0;
	multipathHeader()->pathLength = pathlen;
	multipathHeader()->dataLength = size;
}

void MultiPathPacket::setPayload(const uint8_t *payoad) {
	memcpy(clearData()+sizeof(MultiPathHeaderSt)+sizeof(uint32_t)*multipathHeader()->pathLength, payoad, multipathHeader()->dataLength);
}

void MultiPath::loop() {
	mRecvDups.loop();
}

void MultiPath::send(MultiPathPacket *pkt, bool initHeader, SentStatusHandler handler) {
	MultiPathHeader *header = pkt->multipathHeader();
	// Fill protocol header...
	header->protocol = PROTOCOL_MULTIPATH;
	// Optional fields
	if(initHeader) {
		// Add flags to this packet
		header->flags = 0;
		// If is an ACK i use the last seqno
		header->seqno = ++mLastSequenceNum;
		// Source of this packet
		header->sourceAddress = mPacketBuf->nodeId();
	}
	// Set this class as destination sent callback for retransmisisons
	pkt->setCallback(handler);
    pkt->encryptClearData();
	uint32_t target = header->pathIndex < header->pathLength ? pkt->getPathItem(header->pathIndex) : header->trargetAddress;
    pkt->fill80211((uint8_t *)&target, mPacketBuf->nodeIdPtr());
	//LIB_LOGD(TAG, "MultiPath::send to %06X via %06X path %d/%d seq %d try %d", header->trargetAddress, target, header->pathIndex, header->pathLength, header->seqno, header->flags & MULTIPATH_FLAG_RETRANSMIT_MASK);
	mPacketBuf->send(pkt);
}

/*void MultiPath::send(const uint8_t *data, uint16_t size, uint32_t target, uint32_t *path, uint8_t pathSize, uint8_t port, SentStatusHandler handler) {
	MultiPathPacket *pkt = new MultiPathPacket(this);
	pkt->allocClearData(size, pathSize);
	pkt->multipathHeader()->port = port;
	pkt->multipathHeader()->trargetAddress = target;
	for(int i=0;i<pathSize;i++) pkt->setPathItem(path[i], i);
	pkt->setPayload(data);
	send(pkt, true, handler);
}*/

void MultiPath::send(const uint8_t *data, uint16_t size, const MeshAddress &target, SentStatusHandler handler) {
	MultiPathPacket *pkt = new MultiPathPacket(this);
	pkt->allocClearData(size, target.repeaters.size());
	pkt->multipathHeader()->port = target.port;
	pkt->multipathHeader()->trargetAddress = target.address;
	for(int i=0;i<target.repeaters.size();i++) pkt->setPathItem(target.repeaters[i], i);
	pkt->setPayload(data);
	send(pkt, true, handler);
}

void MultiPath::radioPacketRecv(uint8_t *buf, uint16_t size, uint32_t from, int16_t  rssi) {
	if(size > sizeof(MultiPathHeaderSt)) {
	    MultiPathHeader *header = (MultiPathHeader *)buf;
		uint16_t wsize = sizeof(MultiPathHeaderSt)+header->dataLength+header->pathLength*sizeof(uint32_t);
		LIB_LOGV(TAG, "MultiPath for %06X port %d from %06X with seq %d data %d path %d/%d", header->trargetAddress, header->port, from, header->seqno, header->dataLength, header->pathIndex, header->pathLength);
		if(size >= wsize) {
			if(mRecvDups.checkDuplicateTable(header->sourceAddress, 0, header->seqno)) {
				LIB_LOGE(TAG, "MultiPath duplicated packet received from %06lX with seq %d originator %06lX", from, header->seqno, header->sourceAddress);
				return;
			}

			if(header->pathIndex<header->pathLength) {
				MultiPathPacket *pkt = new MultiPathPacket(this);
				pkt->fromRawData(buf, size);
				pkt->multipathHeader()->pathIndex++;
				send(pkt, false, nullptr);
			} else {
				MeshAddress sourceAddress = MeshAddress(header->port, header->sourceAddress, buf+sizeof(MultiPathHeaderSt), header->pathLength, true);
				sourceAddress.sourceProtocol = MeshAddress::SRC_MULTIPATH;
				this->callReceiveHandler(
					    buf + sizeof(MultiPathHeaderSt) + sizeof(uint32_t) * header->pathLength,   // Payload is at size of the header + size of the path
						header->dataLength,
						sourceAddress,
						rssi);
			}

		} else {
	        LIB_LOGE(TAG, "MultiPath::recv invalid size %d but required %d", size, wsize);
		}
	} else {
        LIB_LOGE(TAG, "MultiPath::recv invalid size %d but required at least %d", size, sizeof(MultiPathHeaderSt));
	}
}

void MultiPath::radioPacketSent(uint8_t status, RadioPacket *pkt) {
	MultiPathPacket *oldpkt = (MultiPathPacket *)pkt;

    if(status) {
        // Handle transmission error onyl with packets with clean data
		MultiPathHeader *header = oldpkt->multipathHeader();
        if(header != nullptr) {
			if((header->flags & MULTIPATH_FLAG_RETRANSMIT_MASK) < MULTIPATH_MAX_RETRANSMISSIONS) {
				MultiPathPacket *newpkt = new MultiPathPacket(this);
				newpkt->fromRawData(pkt->clearData(), pkt->clearDataSize());
				newpkt->multipathHeader()->flags++;
				send(newpkt, false, oldpkt->getCallback());
				return;
			} else {
				LIB_LOGE(TAG, "MultiPath::radioPacketSent transmission error for %06lX via %06lX path %d/%d after %d try", header->trargetAddress, pkt->target8211(), header->pathIndex, header->pathLength, header->flags & 0xF);
			}

        }
	}
	pkt->callCallback(status, pkt);
}

}  // namespace espmeshmesh

