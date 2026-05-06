#include "meshaddress.h"
#include "packetbuf.h"
#include <algorithm>
namespace espmeshmesh {

MeshAddress::MeshAddress(const MeshAddress &other, bool reversed): sourceProtocol(other.sourceProtocol), 
protocolHandle(other.protocolHandle), port(other.port), address(other.address) {
    repeaters = other.repeaters;
    if(reversed) std::reverse(repeaters.begin(), repeaters.end());
}

MeshAddress::MeshAddress(uint16_t port, uint32_t address, const uint8_t *path, uint8_t pathCount, bool reversed): repeaters(std::vector<uint32_t>()), port(port), address(address) {
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

MeshAddress::DataSrc MeshAddress::calcBestProtocol() const {
    if(sourceProtocol != SRC_NONE) {
        return sourceProtocol;
    }
    if(address == MeshAddress::broadCastAddress) {
        return SRC_BROADCAST;
    }
    if(address == MeshAddress::politeBroadcastAddress) {
        return SRC_POLITEBRD;
    }
    if(address == MeshAddress::coordinatorAddress) {
        return SRC_STARPATH;
    }
    if(repeaters.empty()) {
        return SRC_UNICAST;
    }
    return SRC_MULTIPATH;
}

} // namespace espmeshmesh