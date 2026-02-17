#ifdef USE_LINUX
#pragma once

#include "uart.h"

namespace espmeshmesh {

class UartLinux : public Uart {
public:
    UartLinux();
    void setup() override;
    void loop() override;
protected:
    void sendByte(uint8_t data) override;
};

}  // namespace espmeshmesh
#endif