#include "uart.h"
#include "uart_esp32.h"
#include "uart_esp8266.h"
#include "log.h"
#include "crc16.h"
#include "meshaddress.h"
#include "espmeshmesh.h"

#include <memory.h>
#include <functional>

static const char *TAG = "Uart";

namespace espmeshmesh {

Uart::Uart(EspMeshMesh *mesh): mMesh(mesh) {
}

void Uart::sendFramedData(const uint8_t *data, uint16_t length) {
    uint16_t i;
    uint16_t crc16 = 0;
    sendByte(CODE_DATA_START);
    for (i = 0; i < length; i++) {
        if (data[i] == CODE_DATA_ESCAPE || data[i] == CODE_DATA_END || data[i] == CODE_DATA_START) {   
            sendByte(CODE_DATA_ESCAPE);
            crc16 = crc_ibm_byte(crc16, CODE_DATA_ESCAPE);
        }
        sendByte(data[i]);
        crc16 = crc_ibm_byte(crc16, data[i]);
    }
    sendByte(CODE_DATA_END);
    sendByte(crc16 >> 8);
    sendByte(crc16 & 0xFF);
}

void Uart::recvByte(uint8_t byte) {  
    switch (mRecvState) {
    case WAIT_START:
        if (byte == CODE_DATA_START) {
        if (!mFramedBuffer) {
            mFramedBuffer = new uint8_t[mFramedBufferLength];
        }
        memset(mFramedBuffer, 0, mFramedBufferLength);
        mFramedBufferPos = 0;
        mComputedCrc16 = 0;
        mRecvState = WAIT_DATA;
        }
        break;
    case WAIT_DATA:
        if (byte == CODE_DATA_END) {
        mRecvState = WAIT_CRC16_1;
        } else {
        if (byte == CODE_DATA_ESCAPE) {
            mComputedCrc16 = crc_ibm_byte(mComputedCrc16, byte);
            mRecvState = WAIT_ESCAPE;
        } else {
            mComputedCrc16 = crc_ibm_byte(mComputedCrc16, byte);
            mFramedBuffer[mFramedBufferPos++] = byte;
        }
        }
        break;
    case WAIT_ESCAPE:
        if (byte == CODE_DATA_ESCAPE || byte == CODE_DATA_END || byte == CODE_DATA_START) {
        mComputedCrc16 = crc_ibm_byte(mComputedCrc16, byte);
        mFramedBuffer[mFramedBufferPos++] = byte;
        }
        mRecvState = WAIT_DATA;
        break;
    case WAIT_CRC16_1:
        mReceivedCrc16 = uint16_t(byte) << 8;
        mRecvState = WAIT_CRC16_2;
        break;
    case WAIT_CRC16_2:
        mReceivedCrc16 = mReceivedCrc16 | uint16_t(byte);
        if (mComputedCrc16 == mReceivedCrc16) {
        mMesh->handleFrame(mFramedBuffer, mFramedBufferPos, MeshAddress(MeshAddress::SRC_SERIAL, 0, MeshAddress::noAddress), 0);
        } else {
        LIB_LOGE(TAG, "CRC16 mismatch %04X %04X", mComputedCrc16, mReceivedCrc16);
        }
        mRecvState = WAIT_START;
        break;
    }
}

Uart *uartFactory(EspMeshMesh *mesh) {
#if defined(IDF_VER)
    return new UartEsp32(mesh);
#elif defined(ESP8266)
    return new UartEsp8266(mesh);   
#else
    return nullptr;
#endif

}

}