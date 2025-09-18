#pragma once

#include "packetbuf.h"
#include <functional>

namespace espmeshmesh {

typedef std::function<void(uint8_t *data, uint16_t size)> SocketReceiveHandler;
typedef std::function<void(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi)> SocketRecvDatagramHandler;
typedef std::function<void(bool status)> SocketSentStatusHandler;
typedef std::function<void(uint32_t from)> SocketNewConnectionHandler;

class EspMeshMesh;

class SocketDatagram {
public:
    SocketDatagram(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
    ~SocketDatagram();

    uint8_t *data() const { return mData; }
    uint16_t size() const { return mSize; }
    uint32_t from() const { return mFrom; }
    int16_t rssi() const { return mRssi; }
private:
    uint8_t *mData;
    uint16_t mSize;
    uint32_t mFrom;
    int16_t mRssi;
};

class MeshSocket {
private:
    enum SocketProtocol {broadcastProtocol, unicastProtocol, multipathProtocol, politeProtocol, connpathProtocol};
public:
    enum SocketType {
        SOCK_STREAM,  // USe to create a stream socket. 
        SOCK_DGRAM,   // Use to create a datagram socket.
        SOCK_FLOOD    // Use to create a datagram socket that will flood the network.
    };

    enum StatusFlags {
        Closed = 0,        // Socket is closed
        Listening = 1,     // Socket is listening
        Opened = 2,        // Socket is opened
        Connected = 3,     // Connection established, remote peer accepted the connection (only for stream sockets)
    };

    enum ErrorCodes {
        errSuccess = 0,
        errNoParentNetworkAvailable = -1,
        errProtCantBeBinded = -2,
        errRepeatersNotAllowed = -3,
        errCantSendData = -4,
        errAlreadyClosed = -5,
        errTooManyRepeaters = -6,
        errIsNotClosed = -7,
        errIsNotConnected = -8,
        errInvalidTargetAddress = -9,
    };

    /**
     * @brief Broadcast address used to send and receive data to all neighboors.
     */
    static const uint32_t broadCastAddress = UINT32_MAX;
    /**
     * @brief Maximum number of repeaters allowed for a socket
     */
    static const uint8_t maxRepeaters = 16;
    
    /**
     * @brief Create a new socket that will send and receive data from the target. 
     * If the target is the broadcast address, the socket will send and receive data to all neighboors.
     * @param port Port to use for the socket
     * @param target Target to send data to
     * @param type Type of socket
     * @param zero terminated array of addresses of the repeaters  to use for multihop protocols
     */
    MeshSocket(uint8_t port, uint32_t target, uint32_t *repeaters = nullptr);
    /**
     * @brief Destructor
     */
    virtual ~MeshSocket();
    /**
     * @brief Return the target address of the socket
     * @return Target address of the socket
     */
    uint32_t getTargetAddress() const { return mTarget; }
    /**
     * @brief Return true if the target address is the broadcast address
     * @return True if the target address is the broadcast address
     */
    bool isBradcastTarget() const { return mTarget == broadCastAddress; }
    /**
     * @brief Return the status of the socket
     * @return Status of the socket
     */
    StatusFlags status() const;
    /**
     * @brief Listen for incoming connections
     * @return 0 if the socket is listening correctly, otherwise an error code
     */
    uint8_t listen(SocketNewConnectionHandler handler);
    /**
     * @brief Open the socket and allocate the resource to handle its connection.
     * @return 0 if the socket is opened correctly, otherwise an error code
     */
    int8_t open(SocketType type = SOCK_DGRAM);
    /**
     * @brief Send data to the target
     * @param data Data to send
     * @param size Size of the data
     * @param optional callback to receive the sent status information.
     * @return 0 if the data is sent correctly, otherwise an error code
     */
    int8_t send(const uint8_t *data, uint16_t size, SocketSentStatusHandler handler=nullptr);
    /**
     * @brief Set the sent status callback. This callback is called when a packet has been sent from raidio layer. The callback will be called with true argument 
     * if the packet has been sent correctly, otherwise with false.
     * @param handler Sent status callback
     */
    void sentStatusCb(SocketSentStatusHandler handler);
    /**
     * @brief Receive data from the socket the function is not blocking and will fill the buffer with the received data up to size. 
     * If the type of socket is not SOCK_DGRAM or SOCK_FLOOD, the function will return the data coorresponding to one or more received datagrams. 
     * IF the size of the biffer is not enough to contain the first received datagram, the function will return an error code. 
     * If the sieze of buffer is enough, the function will return more the one datagram concatenated. If you need to receive only one datagram, 
     * you can use the recvDatagram function.
     * @param data Data to receive
     * @param size Size of the data
     * @return Number of bytes received or -1 if there is an error
     */
    int8_t recv(uint8_t *data, uint16_t size);
    /**
     * @brief Receive a datagram using the opened socket, the function is not blocking and will return the last received datagram.
     * If the socket is not opened, the function will return an error code, if the type of socket is not SOCK_DGRAM or SOCK_FLOOD, 
     * the function will return an error code.
     * @param data buffer that will contain the received data
     * @param size the maximum size of the data to receive
     * @param from a variable that will contain the source address of the datagram
     * @param rssi a variable that will contain the received signal strength indication
     * @return in case of error, the function will return an error code, otherwise it will return the number of bytes received
     */
    int8_t recvDatagram(uint8_t *data, uint16_t size, uint32_t &from, int16_t &rssi);
    /**
     * @brief Return the number of bytes available to receive
     * @return Number of bytes available to receive
     */
    int16_t recvAvail() const;
    /**
     * @brief Set the receive callback
     * @param handler Receive callback
     */
    void recvCb(SocketReceiveHandler handler);
    /**
     * @brief Set the receive datagram callback
     * @param handler Receive datagram callback
    **/
    void recvDatagramCb(SocketRecvDatagramHandler handler);
    /**
     * @brief Close the socket and release the resources
     */
    uint8_t close();
private:
    void recvFromBroadcast(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
    void recvFromUnicast(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
    void recvFromMultipath(uint8_t *data, uint16_t size, uint32_t from, int16_t rssi, uint8_t *path, uint8_t pathSize);
private:
    EspMeshMesh *mParent{0};
private:
    SocketProtocol mProtocol{broadcastProtocol};
    StatusFlags mStatus{Closed};
    uint8_t mPort{0};
    uint32_t mTarget{0};
    uint32_t *mRepeaters{0};
    uint8_t mRepeatersCount{0};
    bool mIsReversePath{false};
private:
    SocketReceiveHandler mRecvHandler{nullptr};
    SocketRecvDatagramHandler mRecvDatagramHandler{nullptr};
    SocketSentStatusHandler mSentStatusHandler{nullptr};
    SocketNewConnectionHandler mNewConnectionHandler{nullptr};
};

} // namespace espmeshmesh