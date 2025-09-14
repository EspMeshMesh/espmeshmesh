#pragma once

#include "packetbuf.h"
#include <functional>

namespace espmeshmesh {

typedef std::function<void(uint8_t *data, uint16_t size)> SocketReceiveHandler;
typedef std::function<void(bool status)> SocketSentStatusHandler;

class Socket {
    enum SocketDomain {
        SOCKET_BROADCAST,
        SOCKET_UNICAST,
        SOCKET_MULTIHOP,
        SOCKET_POLITE_BROADCAST,
    };
public:
    /**
     * @brief Create a new socket that will send and receive data on the given port and domain.
     * @param port Port to use for the socket
     * @param target Target to send data to
     * @param type Type of socket
     * @param addresses of the repeaters  to use for multihop protocols
     */
    Socket(uint8_t port, uint32_t target, SocketDomain domain, uint32_t repeaters = 0);
    /**
     * @brief Open the socket and allocate the resource to handle its connection.
     * @return 0 if the socket is opened correctly, otherwise an error code
     */
    uint8_t open();
    /**
     * @brief Send data to the target
     * @param data Data to send
     * @param size Size of the data
     * @param optional callback to receive the sent status information.
     * @return 0 if the data is sent correctly, otherwise an error code
     */
    uint8_t send(const uint8_t *data, uint16_t size, SocketSentStatusHandler handler=nullptr);
    /**
     * @brief Set the sent status callback. This callback is called when the has been sent with true argument 
     * if the data has been sent correctly, otherwise with false.
     * @param handler Sent status callback
     */
    void sentStatusCb(SocketSentStatusHandler handler);
    /**
     * @brief Receive data from the socket the function is not blocking and returns the number of bytes
     * available or -1 if ther is an error.
     * @param data Data to receive
     * @param size Size of the data
     * @return Number of bytes received or -1 if there is an error
     */
    int8_t recv(uint8_t *data, uint16_t size);
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
     * @brief Close the socket and release the resources
     */
    uint8_t close();
};

} // namespace espmeshmesh