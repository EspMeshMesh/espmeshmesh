#pragma once
#include "defines.h"
#include <vector>
#include <cstdint>

namespace espmeshmesh {

/**
 * This class is used to represent an address in the mesh network.
 * The address is composed of a port, an address and a list of repeaters.
 * The port is the source or destination port of the protocol used to send or receive data.
 * The address is the source or destination address.
 * The repeaters rapresent the path to the destination or source address.
 */

struct MeshAddress {
    /**
     * @brief Broadcast address used to send and receive data to all neighboors.
     */
    static const uint32_t noAddress = 0;
    static const uint32_t broadCastAddress = UINT32_MAX;

    typedef enum {
        SRC_NONE       = 0x00,
        SRC_BROADCAST  = 0x01,
        SRC_UNICAST    = 0x02,
        SRC_PREROUTED  = 0x03,
        SRC_MULTIPATH  = 0x04,
        SRC_POLITEBRD  = 0x05,
        SRC_CONNPATH   = 0x07,
        SRC_BROADCAST2 = 0x06,
        SRC_FILTER     = 0xfe,
        SRC_SERIAL     = 0xff,
    } DataSrc;
    
    MeshAddress(): repeaters(std::vector<uint32_t>()), port(0), address(0) {}
    MeshAddress(uint8_t port, uint32_t address): repeaters(std::vector<uint32_t>()), port(port), address(address) {}
    MeshAddress(uint8_t port, uint32_t address, std::vector<uint32_t> repeaters): repeaters(repeaters), port(port), address(address) {}
    MeshAddress(uint8_t port, uint32_t address, uint8_t *path, uint8_t pathCount, bool reversed=false);
    MeshAddress(const MeshAddress &other): sourceProtocol(other.sourceProtocol), repeaters(other.repeaters), port(other.port), address(other.address) {}
    bool isBroadcast() const { return address == broadCastAddress; }

    DataSrc sourceProtocol{SRC_NONE};
    std::vector<uint32_t> repeaters;
    uint8_t port{0};
    uint32_t address{0};
};

} // namespace espmeshmesh