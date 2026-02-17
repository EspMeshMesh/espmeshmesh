#include "espmeshmesh.h"
#include "espmeshmesh_impl.h"

namespace espmeshmesh {

EspMeshMesh *EspMeshMesh::singleton = nullptr;

EspMeshMesh *EspMeshMesh::getInstance() { return singleton; }

EspMeshMesh::EspMeshMesh(): mPimpl(new EspMeshMesh::Impl()) {
    if (singleton == nullptr) singleton = this;
}

WifiDrv *EspMeshMesh::getWifiDriver() const {
  return mPimpl->getWifiDriver();
}

bool EspMeshMesh::isCoordinator() const {
  return mPimpl->isCoordinator();
}

bool EspMeshMesh::isBackbone() const {
  return mPimpl->isBackbone();
}

bool EspMeshMesh::isEdge() const {
  return mPimpl->isEdge();
}

EspMeshMesh::NodeType EspMeshMesh::nodeType() const {
    return mPimpl->nodeType();
}

const std::string &EspMeshMesh::hostname() const {
  return mPimpl->hostname();
}

const std::string &EspMeshMesh::fwVersion() const {
  return mPimpl->fwVersion();
}

const std::string &EspMeshMesh::compileTime() const {
  return mPimpl->compileTime();
}

const std::string EspMeshMesh::libVersion() const {
  return mPimpl->libVersion();
}

void EspMeshMesh::uartSendData(const uint8_t *buff, uint16_t len) {
  mPimpl->uartSendData(buff, len);
}

void EspMeshMesh::broadcastSendData(const uint8_t *buff, uint16_t len) {
  mPimpl->broadcastSendData(buff, len);
}

void EspMeshMesh::broadcastSendData(const uint8_t *buff, uint16_t len, uint16_t port) {
  mPimpl->broadcastSendData(buff, len, port);
}

void EspMeshMesh::unicastSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint16_t port) {
  mPimpl->unicastSendData(buff, len, addr, port);
}

void EspMeshMesh::multipathSendData(const uint8_t *buff, uint16_t len, uint32_t addr, uint8_t pathlen, uint32_t *path) {
  mPimpl->multipathSendData(buff, len, addr, pathlen, path);
}

void EspMeshMesh::commandReply(const uint8_t *buff, uint16_t len) {
  mPimpl->commandReply(buff, len);
}

}  // namespace espmeshmesh