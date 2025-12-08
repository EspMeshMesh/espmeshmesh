#include "meshsocket.h"
#include "espmeshmesh.h"
#include "log.h"
#include "broadcast2.h"
#include "unicast.h"
#include "multipath.h"
#include "starpath.h"

#include <functional>
#include <cstring>
#include <list>

#define TAG "espmeshmesh.socket"

namespace espmeshmesh {

SocketDatagram::SocketDatagram(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi): mData(new uint8_t[size]), mSize(size), mFrom(from), mRssi(rssi) {
    memcpy(mData, data, size);
}

SocketDatagram::~SocketDatagram() {
    delete mData;
}

MeshSocket::MeshSocket(uint8_t port): mTarget(MeshAddress(port, bindAllAddress)) {
}

MeshSocket::MeshSocket(const MeshAddress &target): mTarget(target) {
    LIB_LOGV(TAG, "Creating socket port %d target %06lX repeaters %d", mTarget.port, mTarget.address, mTarget.repeaters.size());
}

MeshSocket::~MeshSocket() {
    close();
}

MeshSocket::StatusFlags MeshSocket::status() const {
    return mStatus;
}

uint8_t MeshSocket::listen(SocketNewConnectionHandler handler) {
    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    mNewConnectionHandler = handler;
    mStatus = Listening;

    return errSuccess;
}

int8_t MeshSocket::open(SocketType type) {
    if(mParent == nullptr) {
        mParent = EspMeshMesh::getInstance();
    }

    if(mParent == nullptr) {
        return errNoParentNetworkAvailable;
    }

    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    if(mTarget.repeaters.size() > maxRepeaters) {
        return errTooManyRepeaters;
    }

    if(mTarget.address == MeshAddress::noAddress ) {
        return errInvalidTargetAddress;
    }

    if(mTarget.address == MeshAddress::broadCastAddress || mTarget.address == bindAllAddress) {
        // Broadcast address
        Broadcast2 *broadcast2 = mParent->broadcast2;
        if(broadcast2 == nullptr) {
            return errNoParentNetworkAvailable;
        }
        if(mTarget.repeaters.size() > 0) {
            return errRepeatersNotAllowed;
        }
        bool res = broadcast2->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromProtocol, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsBroadcast = true;
        mStatus = Connected;
        mType = type;
    }
#ifdef ESPMESH_STARPATH_ENABLED
    if(mTarget.address == MeshAddress::coordinatorAddress || mTarget.address == bindAllAddress) {
        StarPathProtocol *starpath = mParent->starpath;
        if(starpath == nullptr) {
            return errNoParentNetworkAvailable;
        }
        bool res = starpath->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromProtocol, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsStarpath = true;
        mStatus = Connected;
        mType = type;
    }
#endif
    if((mTarget.address != MeshAddress::broadCastAddress && mTarget.repeaters.size() == 0) || mTarget.address == bindAllAddress) {
        // Unicast address
        Unicast *unicast = mParent->unicast;
        if(unicast == nullptr) {
            return errNoParentNetworkAvailable;
        }   
        bool res = unicast->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromProtocol, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsUnicast = true;
        mStatus = Connected;
        mType = type;
    } 
    if((mTarget.address != MeshAddress::broadCastAddress && mTarget.repeaters.size() > 0) || mTarget.address == bindAllAddress) {
        // Multipath address
        MultiPath *multipath = mParent->multipath;
        if(multipath == nullptr) {
            return errNoParentNetworkAvailable;
        }
        bool res = multipath->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromProtocol, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsMultipath = true;
        mStatus = Connected;
        mType = type;
    }
    LIB_LOGV(TAG, "Socket opened successfully %d", mStatus);
    return errSuccess;
}

uint8_t MeshSocket::close() {
    if(mStatus == Closed) {
        return errAlreadyClosed;
    }

    if(mIsBroadcast) {
        Broadcast2 *broadcast2 = mParent->broadcast2;
        if(broadcast2) broadcast2->unbindPort(mTarget.port);
        mIsBroadcast = false;
    }
    if(mIsUnicast) {
        Unicast *unicast = mParent->unicast;
        if(unicast) unicast->unbindPort(mTarget.port);
        mIsUnicast = false;
    }
#ifdef ESPMESH_STARPATH_ENABLED
    if(mIsStarpath) {
        StarPathProtocol *starpath = mParent->starpath;
        if(starpath) starpath->unbindPort(mTarget.port);
        mIsStarpath = false;
    }
#endif
    if(mIsMultipath) {
        MultiPath *multipath = mParent->multipath;
        if(multipath) multipath->unbindPort(mTarget.port);
        mIsMultipath = false;
    }

    mStatus = Closed;
    mRecvHandler = nullptr;
    mRecvDatagramHandler = nullptr;
    mSentStatusHandler = nullptr;
    // Delete all the queued datagrams
    while(mRecvDatagrams.size() > 0) {
        delete mRecvDatagrams.front();
        mRecvDatagrams.pop_front();
    }
    mRecvDatagramsSize = 0;
    return 0;
}

int16_t MeshSocket::send(const uint8_t *data, uint16_t size, SentStatusHandler handler) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(handler) {
        LIB_LOGE(TAG, "Socket::send  sent status callback not implemented yet!");
    }

    SocketProtocol protocol = calcProtocolFromTarget(mTarget);
    if(protocol == broadcastProtocol) {
        mParent->broadcast2->send(data, size, mTarget.port, handler ? handler : nullptr);
    } else if(protocol == starpathProtocol) {
#ifdef ESPMESH_STARPATH_ENABLED
        if(!mParent->starpath->send(data, size, mTarget, handler ? handler : nullptr)) {
            return errCantSendData;
        }
#else
        return errCantSendData;
#endif
    } else if(protocol == unicastProtocol) {
        mParent->unicast->send(data, size, mTarget.address, mTarget.port, handler ? handler : nullptr);
    } else if(protocol == multipathProtocol) {
        mParent->multipath->send(data, size, mTarget, handler ? handler : nullptr);
    }
    return errSuccess;
}

#define SENT_CB(handler) [handler](bool status, RadioPacket *pkt) { if(handler) handler(status); }

int16_t MeshSocket::sendDatagram(const uint8_t *data, uint16_t size, MeshAddress target, SocketSentStatusHandler handler) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(target.address == MeshAddress::noAddress) {
        return errInvalidTargetAddress;
    }

    SocketProtocol protocol = calcProtocolFromTarget(target);
    if(protocol == broadcastProtocol) {
        mParent->broadcast2->send(data, size, target.port, SENT_CB(handler));
    } else if(protocol == starpathProtocol) {
#ifdef ESPMESH_STARPATH_ENABLED
        // I use statpath only to send data to the coordinator, the return is done using multipath.
        if(target.address == MeshAddress::coordinatorAddress) {
            if(!mParent->starpath->send(data, size, target, SENT_CB(handler))) {
                return errCantSendData;
            }
        } else {
            mParent->multipath->send(data, size, target, SENT_CB(handler));
        }
#else
        return errCantSendData;
#endif
    } else if(protocol == unicastProtocol) {
        mParent->unicast->send(data, size, target.address, target.port, SENT_CB(handler));
    } else if(protocol == multipathProtocol) {
        uint8_t repeatersCount = target.repeaters.size();
        
        uint32_t *repeatersArray = nullptr;
        if(repeatersCount > 0) {
            repeatersArray = new uint32_t[repeatersCount];
            for(uint8_t i = 0; i < repeatersCount; i++) repeatersArray[i] = target.repeaters[i];
        }

        mParent->multipath->send(
            data, 
            size, 
            target, 
            SENT_CB(handler));

        if(repeatersArray) delete[] repeatersArray;
    }

    return errSuccess;
}

// TODO: Implement Stream recv and receive multiple datagrams
int16_t MeshSocket::recv(uint8_t *data, uint16_t size) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(mRecvDatagrams.size() == 0) {
        return 0;
    }

    SocketDatagram *datagram = mRecvDatagrams.front();
    if(datagram->size() > size) {
        return errBufferTooSmall;
    }

    int16_t datagramSize = datagram->size();
    memcpy(data, datagram->data(), datagram->size());

    mRecvDatagrams.pop_front();
    mRecvDatagramsSize -= datagramSize;
    delete datagram;
    return datagramSize;
}

int16_t MeshSocket::recvDatagram(uint8_t *data, uint16_t size, MeshAddress &from, int16_t &rssi) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(mRecvDatagrams.size() == 0) {
        return 0;
    }

    SocketDatagram *datagram = mRecvDatagrams.front();
    int16_t datagramSize = datagram->size();

    if(datagramSize > size) {
        return errBufferTooSmall;
    }

    memcpy(data, datagram->data(), datagramSize);
    from = datagram->from();
    rssi = datagram->rssi();

    mRecvDatagrams.pop_front();
    mRecvDatagramsSize -= datagramSize;

    delete datagram;

    return datagramSize;
}

int16_t MeshSocket::recvAvail() const {
    return mRecvDatagramsSize;
}

void MeshSocket::recvCb(SocketReceiveHandler handler) {
    mRecvHandler = handler;
}

void MeshSocket::recvDatagramCb(SocketRecvDatagramHandler handler) {
    mRecvDatagramHandler = handler;
}

MeshSocket::SocketProtocol MeshSocket::calcProtocolFromTarget(const MeshAddress &target) {
    if(target.address == MeshAddress::broadCastAddress) {
        return broadcastProtocol;
    }
    if(target.address == MeshAddress::coordinatorAddress) {
        return starpathProtocol;
    }
    if(target.repeaters.empty()) {
        return unicastProtocol;
    }
    return multipathProtocol;
}

void MeshSocket::recvFromProtocol(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, from, rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    } else {
        if(mRecvDatagramsSize + size > mRecvDatagramsMaxSize) {
            LIB_LOGE(TAG, "recvFromBroadcast buffer overflow %d %d", mRecvDatagramsSize, size);
            return;
        }
        SocketDatagram *datagram = new SocketDatagram(data, size, from, rssi);
        mRecvDatagrams.push_back(datagram);
        mRecvDatagramsSize += size;
    }
}

}  // namespace espmeshmesh
