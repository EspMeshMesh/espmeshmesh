#pragma once
#include "packetbuf.h"
#include "meshaddress.h"

#include <functional>
#include <list>
#include <memory>
#include <queue>

namespace espmeshmesh {

typedef std::function<void(const uint8_t *data, uint16_t size)> SocketReceiveHandler;
typedef std::function<void(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi)> SocketRecvDatagramHandler;
typedef std::function<void(bool status)> SocketSentStatusHandler;
typedef std::function<void(uint32_t from)> SocketNewConnectionHandler;

class EspMeshMesh;

class SocketDatagram {
public:
    SocketDatagram(const uint8_t *data, uint16_t size,const MeshAddress &from, int16_t rssi);
    ~SocketDatagram();

    uint8_t *data() const { return mData; }
    uint16_t size() const { return mSize; }
    const MeshAddress &from() const { return mFrom; }
    int16_t rssi() const { return mRssi; }
private:
    uint8_t *mData;
    uint16_t mSize;
    MeshAddress mFrom;
    int16_t mRssi;
};

/**
 * @brief MeshSocket class to replicate the socket operations of the standard library
 * @details This class is used to replicate the socket operations of the standard library.
 * @todo this class register a lot of listeners in the underlying network components, 
 * We have to be sure theat are all removed when the socket is closed and class deleted.
 * In the current version is a work in progress based on current available consumers.
 * THis class is used by:
 * 1. esphome.api component:
 *   - Server that bind on port 6053:
 *    - MeshSocket(MeshSocket::MM_SOCK_STREAM);
 *    - server::bind() Register listener in the underlying network components to handle the incoming connections.
 *    - server::listen() enable the accpet and backlog for the incoming connections.
 *    - server::close() for a listen socket, un bind the port.
 *    - delete server 
 *   - API clients:
 *    - client::MeshSocket(const MeshAddress &{MM_SOCK_STREAM, SRC_CONNPATH, handle, from}); create an already connected socket using from:handle connected path connection.
 *      - The construcotr register the listeners and set seocket Connected.
 *    - server::accept() the server accept is called to retreive this instance of the socket.
 *    - client::close()
 *    - delete client 
 * 
 */
class MeshSocket {
public:
    enum SocketType {
        MM_SOCK_DGRAM,   // Use to create a datagram socket.
        MM_SOCK_STREAM    // Use to create a stream socket.
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
     * @brief Create a new empty socket
     */
    MeshSocket(MeshSocket::SocketType type);

    /**
     * @brief Create a new datagram socket that will bind all protcols for incoming data.
     * @param port Port t bind on all protocols
     */
    MeshSocket(uint16_t port);

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
     * @brief Bind the socket to a port
     * @param port Port to bind the socket to
     * @return 0 if the socket is bound correctly, otherwise an error code
     */
    int8_t bind(uint16_t port);
    /**
     * @brief Listen for incoming connections
     * @return 0 if the socket is listening correctly, otherwise an error code
     */
    uint8_t listen(uint8_t backlogMaxSize);
    /**
     * @brief Listen for incoming connections
     * @return 0 if the socket is listening correctly, otherwise an error code
     */
    uint8_t listen(SocketNewConnectionHandler handler);
    /**
     * @brief Listen for incoming connections
     * @return 0 if the socket is listening correctly, otherwise an error code
     */
    MeshSocket *accept();
    /**
     * @brief Open the socket and allocate the resource to handle its connection.
     * @return 0 if the socket is opened correctly, otherwise an error code
     */
    int8_t open();
    /**
     * @brief Close the socket and release the resources
     */
    uint8_t close();

    /**
     * @brief Send a single datagram to the target node using the appropriate protocol.
     * @param data The data to send.
     * @param size The size of the data.
     * @param target The target address.
     * @param Optional callback to receive the sent status information (true if the packet has been sent correctly, false otherwise).
     *   this callback is prioritary over the callback set the sentStatusCb function. If this parameter is prvided the sentStatusCb
     *   function will not be called for this packet.
     * @return errSuccess if the data is sent correctly, otherwise an error code.
     */
    int16_t send(const uint8_t *data, uint16_t size, SentStatusHandler handler=nullptr);
    /**
     * @brief Overloaded of the sendDatagram function to use a vector of repeaters
     * @param data A buffer containing the data to send
     * @param size The size of the buffer
     * @param target The address of the target node
     * @param optional callback to receive the sent status information (true if the packet has been sent correctly, false otherwise).
     */
    int16_t sendDatagram(const uint8_t *data, uint16_t size, MeshAddress target, SocketSentStatusHandler handler=nullptr);
    /**
     * @brief Receive data from the socket the function is not blocking and will fill the buffer with the received data up to size.
     * If the type of socket is not MM_SOCK_DGRAM or MM_SOCK_FLOOD, the function will return the data coorresponding to one or more received datagrams.
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
     * If the socket is not opened, the function will return an error code, if the type of socket is not MM_SOCK_DGRAM or MM_SOCK_FLOOD,
     * the function will return an error code.
     * @param data buffer that will contain the received data
     * @param size the maximum size of the data to receive
     * @param from a variable that will contain the source address of the datagram
     * @param rssi a variable that will contain the received signal strength indication
     * @return in case of error, the function will return an error code, otherwise it will return the number of bytes received
     */
    int16_t recvDatagram(uint8_t *data, uint16_t size, MeshAddress &from, int16_t &rssi);
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
    void newConnectionForBacklog(uint32_t from);
private:
    void recvFromProtocol(const uint8_t *data, uint16_t size, const MeshAddress &from, int16_t rssi=0);
    void recvFromStreamProtocol(const uint8_t *data, uint16_t size, MeshAddress &from);
    void disconnectFromStreamProtocol();
private:
    void newStreamClient(uint32_t from, uint16_t handle);
private:
    EspMeshMesh *mParent{0};
private:
    bool mIsConnectedPath{false};
    bool mIsBroadcast{false};
#ifdef USE_POLITE_BROADCAST_PROTOCOL
    bool mIsPoliteBroadcast{false};
#endif
    bool mIsUnicast{false};
    bool mIsMultipath{false};
    bool mIsStarpath{false};
private:
    StatusFlags mStatus{Closed};
    MeshAddress mTarget;
    // TODO: Implement MM_SOCK_FLOOD
    SocketType mType{MM_SOCK_DGRAM};
private:
    std::list<MeshSocket *> mAcceptBacklog;
    uint8_t mAcceptBacklogMaxSize{16};
private:
    std::list<SocketDatagram *> mRecvDatagrams;
    uint32_t mRecvDatagramsSize{0};
    uint32_t mRecvDatagramsMaxSize{2048};
private:
    std::queue<uint8_t> mRecvStreamData;
    uint32_t mRecvStreamDataSize{2048};
private:
    SocketReceiveHandler mRecvHandler{nullptr};
    SocketRecvDatagramHandler mRecvDatagramHandler{nullptr};
    SocketSentStatusHandler mSentStatusHandler{nullptr};
    SocketNewConnectionHandler mNewConnectionHandler{nullptr};
public:
    static const char *error2string(ErrorCodes code);
};

} // namespace espmeshmesh

