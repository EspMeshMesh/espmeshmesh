#include "polite.h"
#ifdef USE_POLITE_BROADCAST_PROTOCOL

#include "log.h"
#include "espmeshmesh.h"
#include "packetbuf.h"

#include <cstring>

namespace espmeshmesh {

static const char *TAG = "espmeshmesh.politebroadcast";
PolitePacket::PolitePacket(pktbufSentCbFn cb, void *arg): RadioPacket(cb, arg) {
	_setup();
}

PolitePacket::PolitePacket(uint8_t *data, uint16_t size): RadioPacket(nullptr, nullptr) {
	_setup();
	allocAndCopyClearData(data, size);
}

void PolitePacket::_setup() {
	for(int i=0;i<POLITE_RETX_NUM;i++) mDelays[i] = 0xFF;
	for(int i=0;i<POLITE_RECEIVED_BY;i++) mReceived[i] = 0x00;
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

bool PolitePacket::checkReceived(uint32_t source) {
	// True when i fill the entire array
	int empty = -1;
	for(int i=0;i<POLITE_RECEIVED_BY;i++) {
		if(empty<0 && mReceived[i]==0) empty=i;
		if(mReceived[i]==source) return false;
	}
	if(empty!=-1) {
		mReceived[empty]=source;
		return false;
	}
	return true;
}

void PoliteBroadcastProtocol::setup() {
}

void PoliteBroadcastProtocol::loop() {
	if(mTimeStamp0 != 0) {
		uint32_t now = millis();
		if(EspMeshMesh::elapsedMillis(now, mTimeStamp0) >= 5000) {
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

void PoliteBroadcastProtocol::setReceivedHandler(PoliteBroadcastReceiveHandler h, void *arg) {
	mReceiveHandler = h;
	mReceiveHandlerArg = arg;
}

void PoliteBroadcastProtocol::receiveRadioPacket(uint8_t *data, uint16_t size, uint8_t *fromptr, int16_t rssi) {

	uint32_t from = *(uint32_t *)fromptr;
	if(mState == StateIdle) {
		uint32_t now = millis();
		if(mTimeStamp0 != 0) {
			LIB_LOGD(TAG, "PoliteBroadcastProtocol::receiveRadioPacket StateIdle early request from:%06lX", from);
			return;
		}

		PolitePacket *pkt = new PolitePacket(data, size);
		mTimeStamp0 = mTimeStamp1 = now;
		mOutPkt = pkt;
		mOutPkt->setAutoDelete(false);
		mOutPkt->encryptClearData();
		mOutPkt->calcDelays();
		mOutPkt->checkReceived(from);
		mState = StateWaitEnd;
		// If this packet is for me handle the packet
		LIB_LOGD(TAG, "PoliteBroadcastProtocol::receiveRadioPacket StateIdle from:%06lX seq:%d dst:%06lX src:%06lX", from, mOutPkt->politeHeader()->sequenceNum, mOutPkt->politeHeader()->destAddr, pkt->politeHeader()->sourceAddr);
		if(mReceiveHandler && (mOutPkt->politeHeader()->destAddr==POLITE_DEST_BROADCAST || mOutPkt->politeHeader()->destAddr==mPacketBuf->nodeId()))
			mReceiveHandler(mReceiveHandlerArg, pkt->politePayload(), pkt->politeHeader()->payloadLenght, pkt->politeHeader()->sourceAddr);

	} else if(mState == StateWaitEnd) {
		// Se è una ripetizione del pacchetto attuale:
		PoliteBroadcastHeader *head = (PoliteBroadcastHeader *)data;
		if(head->sourceAddr == mOutPkt->politeHeader()->sourceAddr && head->sequenceNum == mOutPkt->politeHeader()->sequenceNum) {
			LIB_LOGD(TAG, "receiveRadioPacket StateWaitEnd from:%06lX seq:%d dst:0x%06lX src:0x%06lX", from, mOutPkt->politeHeader()->sequenceNum, mOutPkt->politeHeader()->destAddr,  mOutPkt->politeHeader()->sourceAddr);
			if(mOutPkt->checkReceived(from)) {
				// I haave eared the same packet from enuough sorces I can givup.
				_setIdle();
			}
		} else {
			LIB_LOGD(TAG, "PoliteBroadcast: Ignored packet while busy! from:%06lX %06lX!=%06lX %d!=%d", from, head->sourceAddr, mOutPkt->politeHeader()->sourceAddr, head->sequenceNum, mOutPkt->politeHeader()->sequenceNum);
		}
	}
}

void PoliteBroadcastProtocol::send(const uint8_t *data, uint16_t size, uint32_t target) {
	PolitePacket *pkt = new PolitePacket(_packetSentCb, this);
	pkt->setAutoDelete(false);

	pkt->allocClearData(size);
	memcpy(pkt->politePayload(), data, size);

	// Fill protocol header...
	PoliteBroadcastHeader *header = pkt->politeHeader();
	header->protocol = PROTOCOL_POLITEBRD;
	// If is an ACK i use the last seqno
	header->sequenceNum = mLastSequenceNumber++;
	// Len of payload in protocol header
	header->payloadLenght = size;
	// Soure of message
	header->sourceAddr = mPacketBuf->nodeId();
	header->destAddr = target;
	// Prepare packet
	pkt->calcDelays();
	LIB_LOGD(TAG, "PoliteBroadcastProtocol::send seq:%d dst:%06lX src:%06lX", header->sequenceNum, header->destAddr, header->sourceAddr);
	pkt->encryptClearData();
	_sendPkt(pkt);
}

void PoliteBroadcastProtocol::_packetSentCb(void *arg, uint8_t status, RadioPacket *pkt) {
	((PoliteBroadcastProtocol *)arg)->_radioPacketSent(status, (PolitePacket *)pkt);
}

void PoliteBroadcastProtocol::_radioPacketSent(uint8_t status, PolitePacket *pkt) {
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

}


#endif