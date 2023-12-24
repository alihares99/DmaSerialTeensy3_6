/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#pragma once

#include "kinetis.h"
#include "core_pins.h"
#include "HardwareSerial.h"
#include "DMAChannel.h"

// ------------------- change the following if you want ---------------------- //
#define DMA_TX_BUFFER_SIZE          128
#define DMA_RX_BUFFER_SIZE          4096

// ------------------- do not change the rest ------------------------//

#define DMA_MAX_BURST_DATA_TRANSFER 511         // This is the maximum data we are putting into DMA at once

class DmaSerialTeensy : public Stream
{
public:
    explicit DmaSerialTeensy(int serialNo);
    int peek() override;
    void begin(uint32_t baud);
    int available() override;
    int read() override;
    using Print::write;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(char c);
    size_t write(unsigned long n)   { return write((uint8_t)n); }
    size_t write(long n)            { return write((uint8_t)n); }
    size_t write(unsigned int n)    { return write((uint8_t)n); }
    size_t write(int n)             { return write((uint8_t)n); }

private:
    int serialNo;
    int rxPinNo;
    int txPinNo;

    uint8_t* txBuffer = nullptr;
    uint8_t* rxBuffer = nullptr;
    volatile size_t txBufferTail;
    volatile size_t txBufferHead;
    volatile size_t txBufferCount;
    volatile size_t rxBufferTail;

    volatile bool transmitting = false;

    KINETISK_UART_t* uartBaseAddr = nullptr;
    DMAChannel* dmaChannelSend = nullptr;
    DMAChannel* dmaChannelReceive = nullptr;

    struct SerialPin{
        int serialPin_rx;
        int serialPin_tx;
    };

    static const SerialPin serialPins[6][3];

    static void txCompleteCallback1();
    static void txCompleteCallback2();
    static void txCompleteCallback3();
    static void txCompleteCallback4();

    void txIsr();
};

extern DmaSerialTeensy dmaSerial1;
extern DmaSerialTeensy dmaSerial2;
extern DmaSerialTeensy dmaSerial3;
extern DmaSerialTeensy dmaSerial4;

