#include "socket.h"
#include "espmeshmesh.h"
#include "log.h"
#include "broadcast2.h"
#include "unicast.h"
#include "multipath.h"

#include <functional>

#define TAG "espmeshmesh.socket"

namespace espmeshmesh {

SocketDatagram::SocketDatagram(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi): mData(new uint8_t[size]), mSize(size), mFrom(from), mRssi(rssi) {
    mSize = size;
    mData = new uint8_t[mSize];
    memcpy(mData, data, size);
}

SocketDatagram::~SocketDatagram() {
    delete[] mData;
}

Socket::Socket(uint8_t port, uint32_t target, uint32_t *repeaters): mPort(port), mTarget(target) {
    mRepeatersCount = 0;
    while(repeaters[mRepeatersCount] != 0) {
        mRepeatersCount++;
        if(mRepeatersCount > maxRepeaters) {
            break;
        }
    }
    mRepeaters = new uint32_t[mRepeatersCount];
    for(uint8_t i = 0; i < mRepeatersCount; i++) {
        mRepeaters[i] = repeaters[i];
    }
}

Socket::~Socket() {
    delete[] mRepeaters;
}

Socket::StatusFlags Socket::status() const {
    return mStatus;
}

uint8_t Socket::listen(SocketNewConnectionHandler handler) {
    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    mNewConnectionHandler = handler;
    mStatus = Listening;

    return errSuccess;
}

int8_t Socket::open(SocketType type) {
    if(mParent == nullptr) {
        mParent = EspMeshMesh::getInstance();
    }

    if(mParent == nullptr) {
        return errNoParentNetworkAvailable;
    }

    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    if(mRepeatersCount > maxRepeaters) {
        return errTooManyRepeaters;
    }

    if(mTarget == broadCastAddress) {
        // Broadcast address
        Broadcast2 *broadcast2 = mParent->broadcast2;
        if(broadcast2 == nullptr) {
            return errNoParentNetworkAvailable;
        }
        if(mRepeatersCount > 0) {
            return errRepeatersNotAllowed;
        }
        bool res = broadcast2->bindPort(mPort, std::bind(&Socket::recvFromBroadcast, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mProtocol = broadcastProtocol;
        mStatus = Connected;
    } else {
        // Not a broadcast address
        if(mRepeatersCount == 0) {
            // No repeaters --> unicast
            Unicast *unicast = mParent->unicast;
            if(unicast == nullptr) {
                return errNoParentNetworkAvailable;
            }
            bool res = unicast->bindPort(mPort, std::bind(&Socket::recvFromUnicast, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
            if(!res) {
                return errProtCantBeBinded;
            }
            mProtocol = unicastProtocol;
            mStatus = Connected;
        } else {
            // With repeaters --> multipath
            MultiPath *multipath = mParent->multipath;
            if(multipath == nullptr) {
                return errNoParentNetworkAvailable;
            }
            bool res = multipath->bindPort(mPort, std::bind(&Socket::recvFromMultipath, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
            if(!res) {
                return errProtCantBeBinded;
            }
            mProtocol = multipathProtocol;
            mStatus = Connected;
        }
    }

    return errSuccess;
}

int8_t Socket::send(const uint8_t *data, uint16_t size, SocketSentStatusHandler handler) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(handler) {
        LIB_LOGE(TAG, "Socket::send  sent status callback not implemented yet!");
    }

    int8_t err = 0;
    if(mProtocol == broadcastProtocol) {
        err = mParent->broadcast2->send(data, size, mPort);
    } else if(mProtocol == unicastProtocol) {
        err = mParent->unicast->send(data, size, mTarget, mPort);
    } else if(mProtocol == multipathProtocol) {
        err = mParent->multipath->send(data, size, mTarget, mRepeaters, mIsReversePath, mRepeatersCount, mPort);
    }
    if(err) {
        return errCantSendData;
    }
    return 0;
}

void Socket::sentStatusCb(SocketSentStatusHandler handler) {
    // TODO: Implement
}

int8_t Socket::recv(uint8_t *data, uint16_t size) {
    // TODO: Implement
    return 0;
}

int8_t Socket::recvDatagram(uint8_t *data, uint16_t size, uint32_t &from, int16_t &rssi) {
    // TODO: Implement
    return 0;
}

int16_t Socket::recvAvail() const {
    return 0;
}

void Socket::recvCb(SocketReceiveHandler handler) {
    mRecvHandler = handler;
}

void Socket::recvDatagramCb(SocketRecvDatagramHandler handler) {
    mRecvDatagramHandler = handler;
}

uint8_t Socket::close() {
    if(mStatus == Closed) {
        return errAlreadyClosed;
    }
    
    mStatus = Closed;
    mProtocol = broadcastProtocol;
    mRecvHandler = nullptr;
    mRecvDatagramHandler = nullptr;
    mSentStatusHandler = nullptr;
    return 0;
}

void Socket::recvFromBroadcast(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, from, rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    }
}

void Socket::recvFromUnicast(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi) {
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, from, rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    }
}

void Socket::recvFromMultipath(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path, uint8_t pathSize) {
    if(mRepeatersCount != pathSize) {
        memcpy(mRepeaters, path, pathSize * sizeof(uint32_t));
        mRepeatersCount = pathSize;
    }
    if(mRecvDatagramHandler) {
        mRecvDatagramHandler(data, size, from, rssi);
    } else if(mRecvHandler) {
        mRecvHandler(data, size);
    }
}


}  // namespace espmeshmesh