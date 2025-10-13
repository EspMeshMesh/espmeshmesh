#include "meshsocket.h"
#include "espmeshmesh.h"
#include "log.h"
#include "broadcast2.h"
#include "unicast.h"
#include "multipath.h"

#include <functional>
#include <cstring>
#include <list>

#define TAG "espmeshmesh.socket"

namespace espmeshmesh {

SocketDatagram::SocketDatagram(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi): mData(new uint8_t[size]), mSize(size), mFrom(from), mRssi(rssi) {
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
        bool res = broadcast2->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromBroadcast, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsBroadcast = true;
        mStatus = Connected;
        mType = type;
    }
    if((mTarget.address != MeshAddress::broadCastAddress && mTarget.repeaters.size() == 0) || mTarget.address == bindAllAddress) {
        // Unicast address
        Unicast *unicast = mParent->unicast;
        if(unicast == nullptr) {
            return errNoParentNetworkAvailable;
        }   
        bool res = unicast->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromUnicast, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
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
        bool res = multipath->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromMultipath, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
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

    switch(mProtocol) {
        case broadcastProtocol:
            {
                Broadcast2 *broadcast2 = mParent->broadcast2;
                if(broadcast2) broadcast2->unbindPort(mPort);
            }
            break;
        case unicastProtocol:
            {
                Unicast *unicast = mParent->unicast;
                if(unicast) unicast->unbindPort(mPort);
            }
            break;
        case multipathProtocol:
            {
                MultiPath *multipath = mParent->multipath;
                if(multipath) multipath->unbindPort(mPort);
            }
            break;
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

    int8_t err = 0;
    SocketProtocol protocol = calcProtocolFromTarget(mTarget);
    if(protocol == broadcastProtocol) {
        err = mParent->broadcast2->send(data, size, mTarget.port, handler ? handler : mSentStatusHandler);
    } else if(protocol == unicastProtocol) {
        err = mParent->unicast->send(data, size, mTarget.address, mTarget.port, handler ? handler : mSentStatusHandler);
    } else if(protocol == multipathProtocol) {
        err = mParent->multipath->send(data, size, mTarget.address, mTarget.repeaters.data(), mTarget.repeaters.size(), mIsReversePath ? MultiPath::Reverse : MultiPath::Forward, mTarget.port, handler ? handler : mSentStatusHandler);
    }
    if(err) {
        return errCantSendData;
    }
    return 0;
}*/

int16_t MeshSocket::sendDatagram(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, uint32_t *repeaters, SentStatusHandler handler) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(target.address == MeshAddress::noAddress) {
        return errInvalidTargetAddress;
    }

    SocketProtocol protocol = calcProtocolFromTarget(target);
    if(protocol == broadcastProtocol) {
        uint8_t err = mParent->broadcast2->send(data, size, target.port, handler ? handler : mSentStatusHandler);
        if(err) {
            return errCantSendData;
        }
    } else if(protocol == unicastProtocol) {
        uint8_t err = mParent->unicast->send(data, size, target.address, target.port, handler ? handler : mSentStatusHandler);
        if(err) {
            return errCantSendData;
        }
    } else if(protocol == multipathProtocol) {
        uint8_t repeatersCount = target.repeaters.size();
        uint32_t *repeatersArray = nullptr;
        if(repeatersCount > 0) {
            repeatersArray = new uint32_t[repeatersCount];
            for(uint8_t i = 0; i < repeatersCount; i++) repeatersArray[i] = target.repeaters[i];
        }
        uint8_t err = mParent->multipath->send(data, size, target.address, repeatersArray, repeatersCount, mIsReversePath ? MultiPath::Reverse : MultiPath::Forward, target.port, handler ? handler : mSentStatusHandler);
        if(repeatersArray) delete[] repeatersArray;
        if(err) {
            return errCantSendData;
        }
    }

    return 0;
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

int16_t MeshSocket::recvDatagram(uint8_t *data, uint16_t size, uint32_t &from, int16_t &rssi) {
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
    if(target.repeaters.empty()) {
        return unicastProtocol;
    }
    return multipathProtocol;
}

void MeshSocket::recvFromBroadcast(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, MeshAddress(0, from), rssi);
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

void MeshSocket::recvFromUnicast(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, MeshAddress(0, from), rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    } else {
        if(mRecvDatagramsSize + size > mRecvDatagramsMaxSize) {
            LIB_LOGE(TAG, "recvFromUnicast buffer overflow %d %d", mRecvDatagramsSize, size);
            return;
        }
        mRecvDatagrams.push_back(new SocketDatagram(data, size, from, rssi));
        mRecvDatagramsSize += size;
    }
}

void MeshSocket::recvFromMultipath(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t pathSize) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, MeshAddress(0, from, path, pathSize, true), rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    } else {
        if(mRecvDatagramsSize + size > mRecvDatagramsMaxSize) {
            LIB_LOGE(TAG, "recvFromMultipath buffer overflow %d %d", mRecvDatagramsSize, size);
            return;
        }
        mRecvDatagrams.push_back(new SocketDatagram(data, size, from, rssi));
        mRecvDatagramsSize += size;
    }
}


}  // namespace espmeshmesh
