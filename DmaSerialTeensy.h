//
// Created by Ali on ۱۲/۱۱/۲۰۱۹.
//

#ifndef DMA_FOR_TEENSY_DMASERIALTEENSY_H
#define DMA_FOR_TEENSY_DMASERIALTEENSY_H

#include "kinetis.h"
#include "core_pins.h"
#include "HardwareSerial.h"
#include "DMAChannel.h"

// ------------------- change the following if you want ---------------------- //
#define DMA_TX_BUFFER_SIZE          128
#define DMA_RX_BUFFER_SIZE          128

// ------------------- do not change the rest ------------------------//

#define DMA_MAX_BURST_DATA_TRANSFER 511         // This is the maximum data we are putting into DMA at once

class DmaSerialTeensy : public Stream{

private:
    int serialNo;
    int rxPinNo;
    int txPinNo;

    uint8_t txBuffer[DMA_TX_BUFFER_SIZE];
    uint8_t rxBuffer[DMA_RX_BUFFER_SIZE];
    volatile uint32_t txBufferTail;
    volatile uint32_t txBufferHead;
    volatile uint32_t txBufferCount;
    volatile uint32_t rxBufferTail;
    volatile uint32_t rxBufferHead;
    volatile uint32_t rxBufferCount;

    volatile bool transmitting = false;

    KINETISK_UART_t* uartBaseAddr = nullptr;
    DMAChannel* dmaChannelSend = nullptr;
    DMAChannel* dmaChannelReceive = nullptr;

    typedef struct {
        int serialPin_rx;
        int serialPin_tx;
    } SerialPin;

    static const SerialPin serialPins[6][3];

    static void txCompleteCallback1();
    static void rxCompleteCallback1();
    static void txCompleteCallback2();
    static void rxCompleteCallback2();
    static void txCompleteCallback3();
    static void rxCompleteCallback3();
    static void txCompleteCallback4();
    static void rxCompleteCallback4();
    static void rxCompleteCallback5();

    void rxIsr();
    void txIsr();

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
};

extern DmaSerialTeensy dmaSerial1;
extern DmaSerialTeensy dmaSerial2;
extern DmaSerialTeensy dmaSerial3;
extern DmaSerialTeensy dmaSerial4;

#endif //DMA_FOR_TEENSY_DMASERIALTEENSY_H
