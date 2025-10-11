#include "meshaddress.h"
#include "packetbuf.h"
namespace espmeshmesh {

MeshAddress::MeshAddress(uint8_t port, uint32_t address, uint8_t *path, uint8_t pathCount, bool reversed): repeaters(std::vector<uint32_t>()), address(address), port(port) {
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