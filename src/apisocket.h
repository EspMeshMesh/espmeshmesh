#pragma once
#ifdef API_SOCKET_ENABLED
#include <list>

#include "meshaddress.h"

namespace espmeshmesh {

class EspMeshMesh;
class MeshSocket;
class ApiSocket {
public:
    void init(EspMeshMesh *parent);
    void loop();
    void setPendingReplyDelay(uint32_t delay) { mPendingReplyDelay = delay; }
    uint8_t handleFrame(const uint8_t *data, uint16_t len);
private:
    void recvPendingDatagram(uint8_t cmd, uint16_t port);
    void recvDatagram(uint8_t cmd, MeshSocket *socket);
private:
    bool bindSocket(uint16_t port);
    void unbindSocket(uint16_t port);
    bool sendTo(MeshAddress target, const uint8_t *data, uint16_t len);
    bool recvFrom(MeshAddress &from, uint8_t *data, uint16_t len);
private:
    MeshSocket *findMeshSocket(uint16_t port);
    void frameSocketApiReply(uint8_t cmd, uint16_t port, uint8_t status);
    void frameSocketApiSendReply(uint8_t cmd, uint32_t address, uint16_t port, const uint8_t *data, uint16_t len);
    bool decodeFrameSocketApiSendRequest(const uint8_t *data, uint16_t len);
private:
    uint16_t mPendingReplyDelay{1000};
private:
    EspMeshMesh *mParent;
    std::list<MeshSocket *> mBindedSockets;
    bool mPendingReply{false};
    uint16_t mPendingReplyPort{0};
    uint32_t mPendingReplyTimeout{0};

};

} // namespace espmeshmesh
#endif