#pragma once

#include <vector>
#include <cstdint>

namespace espmeshmesh {

struct MeshAddress {
    MeshAddress(): repeaters(std::vector<uint32_t>()), address(0), port(0) {}
    MeshAddress(uint8_t port, uint32_t address): repeaters(std::vector<uint32_t>()), address(address), port(port) {}
    MeshAddress(uint8_t port, uint32_t address, std::vector<uint32_t> repeaters): repeaters(repeaters), address(address), port(port) {}
    MeshAddress(uint8_t port, uint32_t address, uint8_t *path, uint8_t pathCount);
    std::vector<uint32_t> repeaters;
    uint8_t port{0};
    uint32_t address{0};
};

} // namespace espmeshmesh