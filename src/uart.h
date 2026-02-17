#pragma once
#include <cstdint>
#include <functional>

namespace espmeshmesh {

class Uart {
    typedef enum { WAIT_START, WAIT_DATA, WAIT_ESCAPE, WAIT_CRC16_1, WAIT_CRC16_2 } RecvState;
    typedef enum {
      CODE_DATA_START = 0xFD,
      CODE_DATA_END = 0xEF,
      CODE_DATA_ESCAPE = 0xEA
    } SpcialByteCodes;    
public:
    Uart();
    void setUartNum(int uartNum) { mUartNum = uartNum; }
    void setBaudRate(int baudRate) { mBaudRate = baudRate; }
    void setTxBuffer(int txBuffer) { mTxBuffer = txBuffer; }
    void setRxBuffer(int rxBuffer) { mRxBuffer = rxBuffer; }
public:
    virtual void setup() = 0;
    virtual void loop() {}
    virtual void dump_config() {}
public:
    void sendFramedData(const uint8_t *data, uint16_t length);
    void setIncomingFrameCallback(std::function<void(const uint8_t *data, uint16_t length)> callback);
protected:
    void recvByte(uint8_t data);
    virtual void sendByte(uint8_t data) = 0;
protected: // Configuration
    int mUartNum{0};
    int mBaudRate{115200};
    int mTxBuffer{0x600};
    int mRxBuffer{0x600};
    int mFramedBufferLength{0x600};
private: // State
    RecvState mRecvState{WAIT_START};
    uint16_t mComputedCrc16{0};
    uint16_t mReceivedCrc16{0};
    uint8_t *mFramedBuffer{nullptr};
    uint16_t mFramedBufferPos{0};
private:
    std::function<void(const uint8_t *data, uint16_t length)> mIncomingFrameCallback;
};

/*
 * @brief Create a new Uart instance.
 * @return Uart instance
 */
Uart *uartFactory();

}  // namespace espmeshmesh