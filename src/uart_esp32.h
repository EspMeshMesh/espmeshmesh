#ifdef IDF_VER
#pragma once

#include "uart.h"
#include <driver/uart.h>

namespace espmeshmesh {

class UartEsp32 : public Uart {
public:
    UartEsp32(EspMeshMesh *mesh);
public:
    void setup() override;
private:
    void uartTask(void *arg);
    void recvLoop();
    void sendByte(uint8_t data) override;
private:
    QueueHandle_t mUartQueue{nullptr};
};

}  // namespace espmeshmesh
#endif