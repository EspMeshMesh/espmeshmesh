#pragma once

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
    MeshAddress(): repeaters(std::vector<uint32_t>()), address(0), port(0) {}
    MeshAddress(uint8_t port, uint32_t address): repeaters(std::vector<uint32_t>()), address(address), port(port) {}
    MeshAddress(uint8_t port, uint32_t address, std::vector<uint32_t> repeaters): repeaters(repeaters), address(address), port(port) {}
    MeshAddress(uint8_t port, uint32_t address, uint8_t *path, uint8_t pathCount, bool reversed=false);
    std::vector<uint32_t> repeaters;
    uint8_t port{0};
    uint32_t address{0};
};

} // namespace espmeshmesh