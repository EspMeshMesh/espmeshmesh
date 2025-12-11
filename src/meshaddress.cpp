#include "meshaddress.h"
#include "packetbuf.h"
#include <algorithm>
namespace espmeshmesh {

MeshAddress::MeshAddress(const MeshAddress &other, bool reversed): sourceProtocol(other.sourceProtocol), port(other.port), address(other.address) {
    repeaters = other.repeaters;
    if(reversed) std::reverse(repeaters.begin(), repeaters.end());
}

MeshAddress::MeshAddress(uint8_t port, uint32_t address, const uint8_t *path, uint8_t pathCount, bool reversed): repeaters(std::vector<uint32_t>()), port(port), address(address) {
    if(path && pathCount>0) {
        repeaters.resize(pathCount);
        if(!reversed || pathCount == 1) {
            for(uint8_t i = 0; i < pathCount; i++) {
                repeaters[i] = uint32FromBuffer(path + i * sizeof(uint32_t));
            }
        } else {
            for(uint8_t i = 0; i < pathCount; i++) {
                repeaters[i] = uint32FromBuffer(path + (pathCount - i - 1) * sizeof(uint32_t));
            }
        }
    }
}

} // namespace espmeshmesh