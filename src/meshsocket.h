#pragma once
#include "modules.h"

#include "packetbuf.h"
#include "meshaddress.h"
#include <functional>

namespace espmeshmesh {

typedef std::function<void(uint8_t *data, uint16_t size)> SocketReceiveHandler;
typedef std::function<void(uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi)> SocketRecvDatagramHandler;
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
    enum SocketProtocol {unicastProtocol, multipathProtocol, broadcastProtocol, politeProtocol};
public:
    enum SocketType {
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
        errBufferTooSmall = -10,
        errIsNotDatagram = -11,
    };

    /**
     * @brief Maximum number of repeaters allowed for a socket
     */
    static const uint8_t maxRepeaters = 16;
    static const uint32_t bindAllAddress = UINT32_MAX-1;

    /**
     * @brief Create a new datagram socket that will bind all protcols for incoming data.
     * @param port Port t bind on all protocols
     */
    MeshSocket(uint8_t port);

    /**
     * @brief Create a new socket that will send and receive data from the target.
     * If the target is the broadcast address, the socket will send and receive data to all neighboors.
     * @param port Port to use for the socket
     * @param target Target to send data to
     * @param type Type of socket
     * @param zero terminated array of addresses of the repeaters  to use for multihop protocols
     */
    MeshSocket(const MeshAddress &target);
    /**
     * @brief Destructor
     */
    virtual ~MeshSocket();
    /**
     * @brief Return the target address of the socket
     * @return Target address of the socket
     */
    uint32_t getTargetAddress() const { return mTarget.address; }
    /**
     * @brief Return true if the target address is the broadcast address
     * @return True if the target address is the broadcast address
     */
    bool isBroadcastTarget() const { return mTarget.isBroadcast(); }
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
     * @brief Close the socket and release the resources
     */
    uint8_t close();
    /**
     * @brief Send data to the target
     * @param data A buffer containing the data to send
     * @param size The size of the buffer
     * @param Optional callback to receive the sent status information (true if the packet has been sent correctly, false otherwise).
     * this callback is prioritary over the callback set the sentStatusCb function. If this parameter is prvided the sentStatusCb
     * function will not be called for this packet.
     * @return 0 if the data is sent correctly, otherwise an error code
     */
    int16_t send(const uint8_t *data, uint16_t size, SentStatusHandler handler=nullptr);
    /**
     * @brief Overloaded of the sendDatagram function to use a vector of repeaters
     * @param data A buffer containing the data to send
     * @param size The size of the buffer
     * @param target The address of the target node
     * @param port The destination port of this message
     * @param repeaters A list with the addresses of the repeaters to use in multipath protocol. This list must be zero terminated.
     * @param optional callback to receive the sent status information (true if the packet has been sent correctly, false otherwise).
     * this callback is prioritary over the callback set the sentStatusCb function. If this parameter is prvided the sentStatusCb
     * function will not be called for this packet.
     * @return 0 if the data is sent correctly, otherwise an error code
     */
    int16_t sendDatagram(const uint8_t *data, uint16_t size, uint32_t target, uint16_t port, uint32_t *repeaters = nullptr, SentStatusHandler handler=nullptr);
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
    int16_t recv(uint8_t *data, uint16_t size);
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
    int16_t recvDatagram(uint8_t *data, uint16_t size, uint32_t &from, int16_t &rssi);
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
private:
    static SocketProtocol calcProtocolFromTarget(const MeshAddress &target);
private:
    void recvFromBroadcast(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
    void recvFromUnicast(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
    void recvFromMultipath(DataSrc src, uint8_t *data, uint16_t size, uint32_t from, int16_t rssi);
private:
    EspMeshMesh *mParent{0};
private:
    bool mIsBroadcast{false};
    bool mIsUnicast{false};
    bool mIsMultipath{false};
private:
    StatusFlags mStatus{Closed};
    MeshAddress mTarget;
    bool mIsReversePath{false};
    // TODO: Implement SOCK_FLOOD
    SocketType mType{SOCK_DGRAM};
private:
    std::list<SocketDatagram *> mRecvDatagrams;
    uint32_t mRecvDatagramsSize{0};
    uint32_t mRecvDatagramsMaxSize{2048};
private:
    SocketReceiveHandler mRecvHandler{nullptr};
    SocketRecvDatagramHandler mRecvDatagramHandler{nullptr};
    SocketSentStatusHandler mSentStatusHandler{nullptr};
    SocketNewConnectionHandler mNewConnectionHandler{nullptr};
};

} // namespace espmeshmesh

