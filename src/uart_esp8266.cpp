#ifdef ESP8266
#include "uart_esp8266.h"
#include "log.h"

static const char *TAG = "Uart.Esp8266";

namespace espmeshmesh {

UartEsp8266::UartEsp8266(EspMeshMesh *mesh): Uart(mesh) {
}

void UartEsp8266::setup() {
    mHwSerial = &Serial;
    if(mBaudRate > 0) {
        mHwSerial->begin(mBaudRate);
        mHwSerial->setRxBufferSize(mRxBuffer);
    }

    if(mTxBuffer > 0) {
        mUartTxBuffer.resize(mTxBuffer);
    }
}

void UartEsp8266::loop() {
    if(mBaudRate == 0) return;
    int avail = mHwSerial->available();
    if(avail > 0) {
        LIB_LOGV(TAG, "UartEsp8266::loop uart available bytes: %d", avail);
        while (avail--) recvByte((uint8_t) mHwSerial->read());

    }
    if(mUartTxBuffer.filledSpace() > 0) {
        avail = std::min(mHwSerial->availableForWrite() - 8, (int)mUartTxBuffer.filledSpace());
        if(avail > 0) while (avail--) mHwSerial->write(mUartTxBuffer.popByte());
    }
}   

void UartEsp8266::sendByte(uint8_t data) {
    if(mUartTxBuffer.filledSpace() == 0 && mHwSerial->availableForWrite() > 8  ) {
        mHwSerial->write(data);
    } else {
        if(mUartTxBuffer.filledSpace() < mTxBuffer) {
            mUartTxBuffer.pushByte(data);
        } else {
            // Log will write more bytes to the serial
            //LIB_LOGE(TAG, "UartEsp8266::sendByte buffer overflow");            
        }
    }
}

}  // namespace espmeshmesh
#endif