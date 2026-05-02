#include "meshsocket.h"
#include "espmeshmesh.h"
#include "log.h"
#include "broadcast2.h"
#include "unicast.h"
#include "multipath.h"
#include "starpath.h"
#include "connectedpath.h"

#include <functional>
#include <cstring>

#define TAG "espmeshmesh.socket"

namespace espmeshmesh {

SocketDatagram::SocketDatagram(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi): mData(new uint8_t[size]), mSize(size), mFrom(from), mRssi(rssi) {
    memcpy(mData, data, size);
}

SocketDatagram::~SocketDatagram() {
    delete mData;
}


MeshSocket::MeshSocket(MeshSocket::SocketType type): mType(type) {
}

MeshSocket::MeshSocket(uint16_t port): mTarget(MeshAddress(port, bindAllAddress)) {
}

MeshSocket::MeshSocket(const MeshAddress &target): mTarget(target, false) {
    mParent = EspMeshMesh::getInstance();
    if(mParent == nullptr) {
        LIB_LOGE(TAG, "MeshSocket: No meshmesh network available");
        return;
    }

    /**
     * If the protocol is connected path and we have an handle the socker is alreasdy opened.
     * This happens when we accept a new connection.
     */
    if(target.sourceProtocol == MeshAddress::SRC_CONNPATH && target.protocolHandle != 0) {
        mIsConnectedPath = true;
        auto receiveHandler = std::bind(&MeshSocket::recvFromStreamProtocol, this, std::placeholders::_1, std::placeholders::_2, mTarget);
        auto disconnectHandler = std::bind(&MeshSocket::disconnectFromStreamProtocol, this);
        mParent->mConnectedPath->setReceiveCallback(receiveHandler, disconnectHandler, target.address, target.protocolHandle);
        mStatus = Connected;
    }
}

MeshSocket::~MeshSocket() {
    close();
}

MeshSocket::StatusFlags MeshSocket::status() const {
    return mStatus;
}

int8_t MeshSocket::bind(uint16_t port) {
    if(mParent == nullptr) {
        mParent = EspMeshMesh::getInstance();
    }

    if(mParent == nullptr) {
        return errNoParentNetworkAvailable;
    }

    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    mTarget.port = port;
    mTarget.address = bindAllAddress;
    mTarget.repeaters.clear();

    if(mType == MM_SOCK_STREAM) {
        ConnectedPath *connectedPath = mParent->mConnectedPath;
        if(connectedPath == nullptr) {
            return errNoParentNetworkAvailable;
        }
        auto newStreamClient = std::bind(&MeshSocket::newStreamClient, this, std::placeholders::_1, std::placeholders::_2);
        mParent->mConnectedPath->bindPort(newStreamClient, mTarget.port);
        mIsConnectedPath = false;
        mStatus = Listening;
        return errSuccess;
    }

    if(mType == MM_SOCK_DGRAM && (mTarget.address == MeshAddress::broadCastAddress || mTarget.address == bindAllAddress)) {
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
        mStatus = Listening;
    }
#ifdef ESPMESH_STARPATH_ENABLED
    if(mType == MM_SOCK_DGRAM && (mTarget.address == MeshAddress::coordinatorAddress || mTarget.address == bindAllAddress)) {
        StarPathProtocol *starpath = mParent->starpath;
        if(starpath == nullptr) {
            return errNoParentNetworkAvailable;
        }
        bool res = starpath->bindPort(mTarget.port, std::bind(&MeshSocket::recvFromProtocol, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
        if(!res) {
            return errProtCantBeBinded;
        }
        mIsStarpath = true;
        mStatus = Listening;
    }
#endif
    if(mType == MM_SOCK_DGRAM && ((mTarget.address != MeshAddress::broadCastAddress && mTarget.repeaters.size() == 0) || mTarget.address == bindAllAddress)) {
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
        mStatus = Listening;
    } 
    if(mType == MM_SOCK_DGRAM && ((mTarget.address != MeshAddress::broadCastAddress && mTarget.repeaters.size() > 0) || mTarget.address == bindAllAddress)) {
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
        mStatus = Listening;
    }
    LIB_LOGV(TAG, "Socket opened successfully %d", mStatus);
    return errSuccess;
}

uint8_t MeshSocket::listen(uint8_t backlogMaxSize) {
    if(mStatus != Listening) {
        return errIsNotClosed;
    }
    mAcceptBacklogMaxSize = backlogMaxSize;
    mNewConnectionHandler = std::bind(&MeshSocket::newConnectionForBacklog, this, std::placeholders::_1);
    return errSuccess;
}

uint8_t MeshSocket::listen(SocketNewConnectionHandler handler) {
    if(mStatus != Closed) {
        return errIsNotClosed;
    }

    mNewConnectionHandler = handler;
    mStatus = Listening;

    return errSuccess;
}

MeshSocket *MeshSocket::accept() {
    if(mStatus != Listening) {
        return nullptr;
    }
    if(mAcceptBacklog.size() == 0) {
        return nullptr;
    }
    auto sock = mAcceptBacklog.front();
    mAcceptBacklog.pop_front();
    LIB_LOGD(TAG, "accepted new connection from %06X:%d prot %d:%d", sock->mTarget.address, sock->mTarget.port, sock->mTarget.sourceProtocol, sock->mTarget.protocolHandle);
    return sock;
}

int8_t MeshSocket::open() {
    LIB_LOGD(TAG, "opening socket %06X:%d prot %d:%d", mTarget.address, mTarget.port, mTarget.sourceProtocol, mTarget.protocolHandle);
    return this->bind(mTarget.port);
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
    if(mIsConnectedPath && mStatus == Listening) {
        ConnectedPath *connectedPath = mParent->mConnectedPath;
        if(connectedPath) connectedPath->unbindPort(mTarget.port);
        mIsConnectedPath = false;
    }
    if(mIsConnectedPath && mStatus == Connected) {
        ConnectedPath *connectedPath = mParent->mConnectedPath;
        if(connectedPath) connectedPath->closeConnection(mTarget.address, mTarget.protocolHandle);
        mIsConnectedPath = false;
    }

    mRecvHandler = nullptr;
    mRecvDatagramHandler = nullptr;
    mSentStatusHandler = nullptr;
    mNewConnectionHandler = nullptr;

    // Delete all the accepted sockets for stream protocol
    for(auto sock : mAcceptBacklog) delete sock;
    mAcceptBacklog.clear();

    // Delete all the queued stream data
    while(mRecvStreamData.size() > 0) {
        mRecvStreamData.pop();
    }

    // Delete all the queued datagrams
    while(mRecvDatagrams.size() > 0) {
        delete mRecvDatagrams.front();
        mRecvDatagrams.pop_front();
    }
    mRecvDatagramsSize = 0;

    mStatus = Closed;
    return 0;
}

int16_t MeshSocket::send(const uint8_t *data, uint16_t size, SentStatusHandler handler) {
    if(mStatus != Connected) {
        return errIsNotConnected;
    }

    if(handler) {
        LIB_LOGE(TAG, "Socket::send  sent status callback not implemented yet!");
    }

    //LIB_LOGD(TAG, "protocol %d sending %d bytes data to target %06X:%04X", mTarget.sourceProtocol, size, mTarget.address, mTarget.protocolHandle);
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
    } else if(protocol == connectedProtocol) {
        mParent->mConnectedPath->enqueueRadioDataToSource(data, size, mTarget.address, mTarget.protocolHandle);
    }
    return size;
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

    if(mRecvStreamData.size() < size) {
        return errBufferTooSmall;
    }

    for(int i = 0; i < size; i++) {
        data[i] = mRecvStreamData.front();
        mRecvStreamData.pop();
    }

    return size;
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

void MeshSocket::newConnectionForBacklog(uint32_t from) {
    LIB_LOGD(TAG, "newConnectionForBacklog from %06lX", from);
}

MeshSocket::SocketProtocol MeshSocket::calcProtocolFromTarget(const MeshAddress &target) {
    if(target.sourceProtocol == MeshAddress::SRC_CONNPATH) {
        return connectedProtocol;
    }
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

void MeshSocket::recvFromStreamProtocol(const uint8_t *data, uint16_t size, MeshAddress &from) {
    LIB_LOGV(TAG, "recvFromStreamProtocol from %06lX handle %d size %d", from.address, from.protocolHandle, size);
    if(mRecvHandler) {
        mRecvHandler(data, size);
    } else {
        if(mRecvStreamData.size() + size > mRecvStreamDataSize) {
            LIB_LOGE(TAG, "recvFromStreamProtocol buffer overflow %d %d", mRecvStreamDataSize, size);
            return;
        }

        for(int i = 0; i < size; i++) {
            mRecvStreamData.push(data[i]);
        }
    }
}

void MeshSocket::disconnectFromStreamProtocol() {
    LIB_LOGI(TAG, "disconnectFromStreamProtocol");
    mStatus = Closed;
}

void MeshSocket::newStreamClient(uint32_t from, uint16_t handle) {
    LIB_LOGD(TAG, "newStreamClient from %06lX handle %d", from, handle);
    mAcceptBacklog.push_back(new MeshSocket(MeshAddress(MeshAddress::SRC_CONNPATH, handle, from, true)));
}

}  // namespace espmeshmesh
