#include "polite.h"
#ifdef USE_POLITE_BROADCAST_PROTOCOL

#include "log.h"
#include "espmeshmesh.h"
#include "packetbuf.h"

#include <cstring>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.politebroadcast";
PolitePacket::PolitePacket(PacketBufProtocol * owner, SentStatusHandler cb): RadioPacket(owner, cb) {
	_setup();
}

PolitePacket::PolitePacket(uint8_t *data, uint16_t size): RadioPacket(nullptr, nullptr) {
	_setup();
	allocAndCopyClearData(data, size);
}

void PolitePacket::_setup() {
	for(int i=0;i<POLITE_RETX_NUM;i++) mDelays[i] = 0xFF;
}

void PolitePacket::allocClearData(uint16_t size) {
	RadioPacket::allocClearData(size+sizeof(PoliteBroadcastHeaderSt));
}

void PolitePacket::calcDelays() {
	uint8_t lastdelay = 0;
	for(int i=0;i<POLITE_RETX_NUM; i++) {
		uint8_t rnd = POLITE_RANDOM_SLOT(POLITE_RETX_SLOTS);
		mDelays[i] = rnd+i*POLITE_RETX_SLOTS;
	}
	LIB_LOGV(TAG, "PolitePacket::calcDelays delays: %d %d %d", mDelays[0], mDelays[1], mDelays[2]);
}

bool PolitePacket::checkDelay(uint32_t elapsed) {
	// Controllo se devo inviare la prossima ripetiziono
	for(int i=0;i<POLITE_RETX_NUM;i++) {
		if(mDelays[i] != 0xFF && elapsed > mDelays[i]*POLITE_SLOT_DURATION_MS) {
			mDelays[i] = 0xFF;
			return true;
		}
	}
	return false;
}

bool PolitePacket::incrementReceived() {
	mReceived++;
	if(mReceived >= POLITE_RECEIVED_BY) {
		return true;
	}
	return false;
}

PoliteBroadcastProtocol::PoliteBroadcastProtocol(PacketBuf *pbuf, ReceiveHandler rx_fn): PacketBufProtocol(pbuf, rx_fn, MeshAddress::SRC_POLITEBRD) {
	for(int i=0;i<POLITE_PAST_MAX;i++) { mPastSenders[i].timestamp = 0; }
}

void PoliteBroadcastProtocol::loop() {
	if(mTimeStamp0 != 0) {
		uint32_t now = millis();
		if(EspMeshMesh::elapsedMillis(now, mTimeStamp0) >= POLITE_GUARDTIME_MS*2) {
			_clearOlderPastSenders();
			mTimeStamp0 = 0;
		}
	}

	if(mState == StateWaitEnd) {
		uint32_t now = millis();
		uint32_t elapsed = EspMeshMesh::elapsedMillis(now, mTimeStamp1);
		if(mOutPkt->checkDelay(elapsed)) {
			_sendRaw();
		} else if(elapsed > POLITE_GUARDTIME_MS) {
			_setIdle();
		}
	}

}  // namespace espmeshmesh

void PoliteBroadcastProtocol::radioPacketRecv(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
	PoliteBroadcastHeader *head = (PoliteBroadcastHeader *)data;
	if(mState == StateIdle) {
		uint32_t now = millis();
		if(_checkPastSenders(head->sourceAddr, head->sequenceNum)) {
			LIB_LOGVV(TAG, "PoliteBroadcastProtocol::receiveRadioPacket StateIdle already handled this packet: %06lX seq %d", head->sourceAddr, head->sequenceNum);
			return;
		}
		if(mTimeStamp0 != 0) {
			LIB_LOGW(TAG, "PoliteBroadcastProtocol::receiveRadioPacket StateIdle early request from:%06lX", from);
			return;
		}

		_addPastSender(head->sourceAddr, head->sequenceNum);

		PolitePacket *pkt = new PolitePacket(data, size);
		mTimeStamp0 = mTimeStamp1 = now;
		mOutPkt = pkt;
		mOutPkt->setAutoDelete(false);
		mOutPkt->encryptClearData();
		mOutPkt->calcDelays();
		mOutPkt->incrementReceived();
		mState = StateWaitEnd;
		// If this packet is for me handle the packet
		LIB_LOGD(TAG, "receiveRadioPacket: StateIdle, from:%06lX seq:%d %06lX->%06lX", from, mOutPkt->politeHeader()->sequenceNum, mOutPkt->politeHeader()->destAddr, pkt->politeHeader()->sourceAddr);
		if(mOutPkt->politeHeader()->destAddr==MeshAddress::politeBroadcastAddress || mOutPkt->politeHeader()->destAddr==mPacketBuf->nodeId()) {
			MeshAddress sourceAddress = MeshAddress(mOutPkt->politeHeader()->port, pkt->politeHeader()->sourceAddr);
			sourceAddress.sourceProtocol = MeshAddress::SRC_POLITEBRD;
			callReceiveHandler(pkt->politePayload(), pkt->politeHeader()->payloadLenght, sourceAddress, rssi);
		}

	} else if(mState == StateWaitEnd) {
		// Se è una ripetizione del pacchetto attuale:
		if(head->sourceAddr == mOutPkt->politeHeader()->sourceAddr && head->sequenceNum == mOutPkt->politeHeader()->sequenceNum) {
			if(mOutPkt->incrementReceived()) {
				// I haave eared the same packet from enuough sorces I can givup.
				LIB_LOGD(TAG, "receiveRadioPacket: Received enough packets, giving up");
				_setIdle();
			}
			if(mOutPkt) LIB_LOGD(TAG, "receiveRadioPacket: StateWaitEnd, from:%06lX seq:%d %06lX->%06lX recv %d", from, mOutPkt->politeHeader()->sequenceNum, mOutPkt->politeHeader()->destAddr,  mOutPkt->politeHeader()->sourceAddr, mOutPkt->received());
		} else {
			LIB_LOGW(TAG, "receiveRadioPacket: Ignored packet while busy! from:%06lX %06lX!=%06lX %d!=%d", from, head->sourceAddr, mOutPkt->politeHeader()->sourceAddr, head->sequenceNum, mOutPkt->politeHeader()->sequenceNum);
		}
	}
}

void PoliteBroadcastProtocol::send(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, SentStatusHandler handler) {
	PolitePacket *pkt = new PolitePacket(this);
	pkt->setAutoDelete(false);

	pkt->allocClearData(size);
	memcpy(pkt->politePayload(), data, size);

	// Fill protocol header...
	PoliteBroadcastHeader *header = pkt->politeHeader();
	header->protocol = PROTOCOL_POLITEBRD;
	header->port = port;
	// If is an ACK i use the last seqno
	header->sequenceNum = mLastSequenceNumber++;
	// Len of payload in protocol header
	header->payloadLenght = size;
	// Soure of message
	header->sourceAddr = mPacketBuf->nodeId();
	header->destAddr = target;
	// Prepare packet
	pkt->calcDelays();
	pkt->setCallback(handler);
	pkt->encryptClearData();
	_sendPkt(pkt);
}

void PoliteBroadcastProtocol::_sendPkt(PolitePacket *pkt) {
	if(mState == StateIdle) {
		mOutPkt = pkt;
		mTimeStamp0 = mTimeStamp1 = millis();
		mState = StateWaitEnd;
		_sendRaw();
	} else {
		mOutPkts.push_back(pkt);
	}
}

void PoliteBroadcastProtocol::_sendRaw() {
	mOutPkt->fill80211(nullptr, mPacketBuf->nodeIdPtr());
	mPacketBuf->send(mOutPkt);
}

void PoliteBroadcastProtocol::_setIdle(void) {
	if(mState == StateIdle) return;
	LIB_LOGD(TAG, "_setIdle");
	mOutPkt->callCallback(false, mOutPkt);
	delete mOutPkt;
	mOutPkt = nullptr;

	mState = StateIdle;
	if(mOutPkts.size()) {
		mOutPkt = mOutPkts.front();
		mOutPkts.pop_front();
		mTimeStamp1 = millis();
		mState = StateWaitEnd;
		_sendRaw();
	}
}

bool PoliteBroadcastProtocol::_checkPastSenders(uint32_t from, uint16_t sequenceNum) {
	for(int i=0;i<POLITE_PAST_MAX;i++) {
		if(mPastSenders[i].from == from && mPastSenders[i].sequenceNum == sequenceNum) {
			mPastSenders[i].timestamp = millis();
			return true;
		}
	}
	return false;
}

void PoliteBroadcastProtocol::_addPastSender(uint32_t from, uint16_t sequenceNum) {
	for(int i=0;i<POLITE_PAST_MAX;i++) {
		if(mPastSenders[i].timestamp == 0 || mPastSenders[i].from == from) {
			mPastSenders[i].timestamp = millis();
			mPastSenders[i].from = from;
			mPastSenders[i].sequenceNum = sequenceNum;
			return;
		}
	}
}

void PoliteBroadcastProtocol::_clearOlderPastSenders() {
	for(int i=0;i<POLITE_PAST_MAX;i++) {
		uint32_t now = millis();
		if(EspMeshMesh::elapsedMillis(now, mPastSenders[i].timestamp) > POLITE_PAST_RETAIN_TIME) {
			mPastSenders[i].timestamp = 0;
			mPastSenders[i].from = 0;
			mPastSenders[i].sequenceNum = 0;
		}
	}
}

void PoliteBroadcastProtocol::radioPacketSent(uint8_t status, RadioPacket *pkt) {
	PolitePacket *oldpkt = (PolitePacket *) pkt;
	oldpkt->callCallback(status, pkt);
}

}


#endif
