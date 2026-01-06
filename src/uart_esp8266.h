#ifdef ESP8266
#pragma once

#include "uart.h"
#include "memringbuffer.h"
#include <HardwareSerial.h>

namespace espmeshmesh {

class UartEsp8266 : public Uart {
public:
    UartEsp8266(EspMeshMesh *mesh);
    void setup() override;
    void loop() override;
    void sendByte(uint8_t data) override;
private:
    HardwareSerial *mHwSerial{nullptr};
    MemRingBuffer mUartTxBuffer;
};

}  // namespace espmeshmesh
#endif