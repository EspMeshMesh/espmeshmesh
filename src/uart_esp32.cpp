#ifdef IDF_VER
#include "uart_esp32.h"

#include "log.h"
#include "crc16.h"
#include "meshaddress.h"
#include "espmeshmesh.h"

#include <memory.h>
#include <functional>

#include <esp_intr_alloc.h>

static const char *TAG = "Uart.Esp32";

namespace espmeshmesh {

UartEsp32::UartEsp32(EspMeshMesh *mesh): Uart(mesh) {
}

void UartEsp32::setup() {
    esp_err_t err;

    err = uart_set_pin((uart_port_t)mUartNum, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        LIB_LOGE(TAG, "uart_set_pin failed");
        return;
    }

    QueueHandle_t uart_queue;
    err = uart_driver_install((uart_port_t)mUartNum, mTxBuffer, mRxBuffer, 10, &mUartQueue, 0);
    if (err != ESP_OK) {
        LIB_LOGE(TAG, "uart_driver_install failed");
        return;
    }

    const uart_config_t uartConfig = {
        .baud_rate = mBaudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    err = uart_param_config((uart_port_t)mUartNum, &uartConfig);
    if (err != ESP_OK) {
        LIB_LOGE(TAG, "uart_param_config failed");
        return;
    }

    err = uart_enable_rx_intr((uart_port_t)mUartNum);
    if (err != ESP_OK) {
        LIB_LOGE(TAG, "uart_enable_rx_intr failed");
        return;
    }


    xTaskCreate([](void *arg) {
        UartEsp32 *uart = (UartEsp32 *)arg;
        uart->uartTask(arg);
    }, "uartTask", 4096, this, 10, nullptr);
}

void UartEsp32::uartTask(void *arg) {
    uart_event_t event;
    while (true) {
        if(xQueueReceive(mUartQueue, &event, pdMS_TO_TICKS(100))) {
            switch(event.type) {
            case UART_DATA:
                recvLoop();
                break;
            case UART_FIFO_OVF:
                xQueueReset(mUartQueue);
                LIB_LOGE(TAG, "UART_FIFO_OVF");
                break;
            case UART_BUFFER_FULL:
                uart_flush_input((uart_port_t)mUartNum);
                xQueueReset(mUartQueue);
                LIB_LOGE(TAG, "UART_BUFFER_FULL");
                break;
            default:
                LIB_LOGW(TAG, "UART_EVENT_UNKNOWN %d", event.type);
                break;
            }
        }
    }
}

void UartEsp32::recvLoop() {
    size_t avail;
    esp_err_t err = uart_get_buffered_data_len((uart_port_t)mUartNum, &avail);
    if (err != ESP_OK) {
        LIB_LOGE(TAG, "uart_get_buffered_data_len failed");
        return;
    }
    if(avail > 0) {
    uint8_t byte;
        while ((err = uart_read_bytes((uart_port_t)mUartNum, &byte, 1, 0)) > 0) {
            recvByte(byte);
        }
    }
    if(err < 0) {
        LIB_LOGE(TAG, "uart_read_bytes failed");
        return;
    }
}

void UartEsp32::sendByte(uint8_t data) {
    uart_write_bytes((uart_port_t)mUartNum, (const char *)&data, 1);
}

}
#endif