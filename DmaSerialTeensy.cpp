/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerialTeensy.h"
#include "Arduino.h"
#include <cstring>

#define MIN(x, y)  ((x) > (y) ? (y) : (x))
#define MAX(x, y)  ((x) > (y) ? (x) : (y))

#ifdef __MK66FX1M0__ // teensy 3.6
const DmaSerialTeensy::SerialPin DmaSerialTeensy::serialPins[6][3] = {
    {{-1, -1}, {-1, -1} , {-1, -1}}, // Serial is ignored
    {{0, 1}, {27, 26} , {21, 5}}, // Serial1
    {{9, 10}, {59, 58} , {-1, -1}}, // Serial2
    {{7, 8}, {-1, -1} , {-1, -1}}, // Serial3
    {{31, 32}, {63, 62} , {-1, -1}}, // Serial4
    {{34, 33}, {-1, -1} , {-1, -1}}, // Serial5
};

DmaSerialTeensy dmaSerial1(1);
DmaSerialTeensy dmaSerial2(2);
DmaSerialTeensy dmaSerial3(3);
DmaSerialTeensy dmaSerial4(4);

#endif

void DmaSerialTeensy::txCompleteCallback1() {dmaSerial1.txIsr();}
void DmaSerialTeensy::rxCompleteCallback1() {dmaSerial1.rxIsr();}
void DmaSerialTeensy::txCompleteCallback2() {dmaSerial2.txIsr();}
void DmaSerialTeensy::rxCompleteCallback2() {dmaSerial2.rxIsr();}
void DmaSerialTeensy::txCompleteCallback3() {dmaSerial3.txIsr();}
void DmaSerialTeensy::rxCompleteCallback3() {dmaSerial3.rxIsr();}
void DmaSerialTeensy::txCompleteCallback4() {dmaSerial4.txIsr();}
void DmaSerialTeensy::rxCompleteCallback4() {dmaSerial4.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback5() {/*dmaSerial5.rxIsr(); // not supported yet*/}

DmaSerialTeensy::DmaSerialTeensy(int serialNo)
    : serialNo(serialNo)
{
    if (serialNo >= 1 && serialNo <= 5) {
        // setting the default serial pins:
        rxPinNo = serialPins[serialNo][0].serialPin_rx;
        txPinNo = serialPins[serialNo][0].serialPin_tx;
    }
    switch (serialNo) {
        case 1:
            uartBaseAddr = &KINETISK_UART0;
            break;
        case 2:
            uartBaseAddr = &KINETISK_UART1;
            break;
        case 3:
            uartBaseAddr = &KINETISK_UART2;
            break;
        case 4:
            uartBaseAddr = &KINETISK_UART3;
            break;
        case 5:
            // Serial 5 only works for receptions. no DMA for sending.
            uartBaseAddr = &KINETISK_UART4;
            break;
    }

}


void DmaSerialTeensy::begin(uint32_t baud) {
	
	if (!txBuffer) {
		txBuffer = new uint8_t[DMA_TX_BUFFER_SIZE];
	}
	if (!rxBuffer) {
		rxBuffer = new uint8_t[DMA_RX_BUFFER_SIZE];
	}

    if (!dmaChannelSend) {
        void (*txIsr)();
        uint8_t dmamuxSource;
        switch (serialNo) {
            case 1:
                dmamuxSource = DMAMUX_SOURCE_UART0_TX;
                txIsr = txCompleteCallback1;
                break;
            case 2:
                dmamuxSource = DMAMUX_SOURCE_UART1_TX;
                txIsr = txCompleteCallback2;
                break;
            case 3:
                dmamuxSource = DMAMUX_SOURCE_UART2_TX;
                txIsr = txCompleteCallback3;
                break;
            case 4:
                dmamuxSource = DMAMUX_SOURCE_UART3_TX;
                txIsr = txCompleteCallback4;
                break;
            case 5:
                // Serial 5 is either capable of reception or transmission through DMA.
                // Current cofig is for reception.
                dmamuxSource = 255;
                break;
            default:
                return;
        }
        if (dmamuxSource != 255ui) {
            dmaChannelSend = new DMAChannel();
            dmaChannelSend->destination(uartBaseAddr->D);
            // source is not configured here
            dmaChannelSend->triggerAtHardwareEvent(dmamuxSource);
            dmaChannelSend->transferSize(1); // length of each datum is 1 byte
            dmaChannelSend->attachInterrupt(txIsr);
            dmaChannelSend->interruptAtCompletion();
            dmaChannelSend->disableOnCompletion();
            // not enabled here
        }
    }
    if (!dmaChannelReceive) {
        void (*rxIsr)();
        uint8_t dmamuxSource;
        switch (serialNo) {
            case 1:
                dmamuxSource = DMAMUX_SOURCE_UART0_RX;
                rxIsr = rxCompleteCallback1;
                break;
            case 2:
                dmamuxSource = DMAMUX_SOURCE_UART1_RX;
                rxIsr = rxCompleteCallback2;
                break;
            case 3:
                dmamuxSource = DMAMUX_SOURCE_UART2_RX;
                rxIsr = rxCompleteCallback3;
                break;
            case 4:
                dmamuxSource = DMAMUX_SOURCE_UART3_RX;
                rxIsr = rxCompleteCallback4;
                break;
            case 5:
                dmamuxSource = DMAMUX_SOURCE_UART4_RXTX;
                rxIsr = rxCompleteCallback5;
                break;
            default:
                return;
        }
        dmaChannelReceive = new DMAChannel();
        dmaChannelReceive->source(uartBaseAddr->D);
        dmaChannelReceive->destinationBuffer(rxBuffer, DMA_RX_BUFFER_SIZE);
        dmaChannelReceive->triggerAtHardwareEvent(dmamuxSource);
        // no need for rx Interrupt:
        // dmaChannelReceive->attachInterrupt(rxIsr);
        // dmaChannelReceive->interruptAtCompletion();
        dmaChannelReceive->enable();
    }

    uint32_t divisor;
    switch (serialNo) {
        case 1:
            SIM_SCGC4 |= SIM_SCGC4_UART0;	// turn on clock
            divisor = BAUD2DIV(baud);    // baudrate to divisor
            break;
        case 2:
            SIM_SCGC4 |= SIM_SCGC4_UART1;	// turn on clock
            divisor = BAUD2DIV2(baud);      // baudrate to divisor
            break;
        case 3:
            SIM_SCGC4 |= SIM_SCGC4_UART2;	// turn on clock
            divisor = BAUD2DIV3(baud);      // baudrate to divisor
            break;
        case 4:
            SIM_SCGC4 |= SIM_SCGC4_UART3;	// turn on clock
            divisor = BAUD2DIV3(baud);      // baudrate to divisor
            break;
        case 5:
            SIM_SCGC1 |= SIM_SCGC1_UART4;	// turn on clock
            divisor = BAUD2DIV3(baud);      // baudrate to divisor
            break;
    }

    txBufferTail = 0;
    txBufferHead = 0;
    txBufferCount = 0;
    rxBufferTail = 0;
    // rxBufferHead = 0;

    // set the rx and tx pins:
    switch (rxPinNo) {
        case 0:  CORE_PIN0_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        case 21: CORE_PIN21_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        #if defined(KINETISL)
        case 3:  CORE_PIN3_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(2); break;
		case 25: CORE_PIN25_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(4); break;
        #endif
        #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        case 27: CORE_PIN27_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        #endif
        case 9: CORE_PIN9_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        #if defined(__MK20DX128__) || defined(__MK20DX256__)    // T3.0, T3.1, T3.2
            case 26: CORE_PIN26_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        #elif defined(__MK64FX512__) || defined(__MK66FX1M0__)  // T3.5 or T3.6
        case 59: CORE_PIN59_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        #endif
        case 7: CORE_PIN7_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        case 31: CORE_PIN31_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        case 63: CORE_PIN63_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
        case 34: CORE_PIN34_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_PFE | PORT_PCR_MUX(3); break;
    }
    switch (txPinNo) {
        case 1:  CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        case 5:  CORE_PIN5_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        #if defined(KINETISL)
        case 4:  CORE_PIN4_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(2); break;
		case 24: CORE_PIN24_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(4); break;
        #endif
        #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
        case 26: CORE_PIN26_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        #endif
        case 10: CORE_PIN10_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        #if defined(__MK20DX128__) || defined(__MK20DX256__)    // T3.0, T3.1, T3.2
            case 31: CORE_PIN31_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        #elif defined(__MK64FX512__) || defined(__MK66FX1M0__)  // T3.5 or T3.6
        case 58: CORE_PIN58_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        #endif
        case 8: CORE_PIN8_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        case 32: CORE_PIN32_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        case 62: CORE_PIN62_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
        case 33: CORE_PIN33_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); break;
    }

    // disable transmitter and receiver
    uartBaseAddr->C2 &= ~(UART_C2_TE | UART_C2_RE);
    // Configure the UART for 8-bit mode, no parity. All default:
    uartBaseAddr->C1 = 0;
    // write baudrate:
    if (divisor < 32) divisor = 32;
    uartBaseAddr->BDH = (divisor >> 13) & 0x1F;
    uartBaseAddr->BDL = (divisor >> 5) & 0xFF;
    uartBaseAddr->C4 = divisor & 0x1F;

    uartBaseAddr->C2 |= UART_C2_TIE; // dma enable. Enables S1[TDRE] to generate interrupt requests or DMA transfer requests, based on the state of C5[TDMAS].
    uartBaseAddr->C2 |= UART_C2_RIE; // Enables S1[RDRF] to generate interrupt requests or DMA transfer requests, based on the state of C5[RDMAS]
    uartBaseAddr->C5 |= UART_C5_TDMAS;
    uartBaseAddr->C5 |= UART_C5_RDMAS;
    uartBaseAddr->PFIFO = 0;
    /*UART0_TWFIFO = 2; // tx watermark, causes S1_TDRE to set
    UART0_RWFIFO = 4; // rx watermark, causes S1_RDRF to set
    UART0_PFIFO = UART_PFIFO_TXFE | UART_PFIFO_RXFE;*/

    // enable transmitter and receiver
    uartBaseAddr->C2 |= (UART_C2_TE | UART_C2_RE);
}

int DmaSerialTeensy::available() {
    int biter = dmaChannelReceive->TCD->BITER;
    int citer = dmaChannelReceive->TCD->CITER;
    int csr = dmaChannelReceive->TCD->CSR;
    if (csr & 0x80) { // done so Rx buffer is full
        if (rxBufferTail == 0) return 0;
        else return DMA_RX_BUFFER_SIZE - rxBufferTail;
    }
    else {
        // our version of buffer indexes are not update
        int head = biter - citer;
        if (head >= rxBufferTail) return head - rxBufferTail;
        else return head - rxBufferTail + DMA_RX_BUFFER_SIZE;
    }
}

int DmaSerialTeensy::read() {
    uint8_t c = rxBuffer[rxBufferTail++];
    if (rxBufferTail >= DMA_RX_BUFFER_SIZE)
        rxBufferTail -= DMA_RX_BUFFER_SIZE;
    return c;
}

int DmaSerialTeensy::peek() {
    return rxBuffer[rxBufferTail];
}

size_t DmaSerialTeensy::write(uint8_t c) {
    write(&c, 1);
    return 1;
}

size_t DmaSerialTeensy::write(char c) {
    return write((uint8_t *)&c, 1);
}

size_t DmaSerialTeensy::write(const uint8_t *p, size_t len) {

    int index = 0;
    while (index < len) {

        // wait until there is free space in the buffer:
        while (DMA_TX_BUFFER_SIZE - txBufferCount /* free size */ == 0);

        // get a chunk of data to add to the buffer
        int chunkSize = MIN(len - index, DMA_TX_BUFFER_SIZE - txBufferCount);

        // copy the data to the buffer:
        int s1 = MIN(chunkSize, DMA_TX_BUFFER_SIZE - txBufferHead);
        int s2 = chunkSize - s1;
        memcpy(&txBuffer[txBufferHead], &p[index], s1);
        if (s2 > 0)
            memcpy(&txBuffer[0], &p[index + s1], s2);
        index += chunkSize;

        // move the head:
        txBufferCount += chunkSize;
        txBufferHead += chunkSize;
        if (txBufferHead >= DMA_TX_BUFFER_SIZE)
            txBufferHead -= DMA_TX_BUFFER_SIZE;

        // start transmitting from the tail:
        if (!transmitting) {
            transmitting = true;
            __disable_irq()
            int count = MIN(DMA_TX_BUFFER_SIZE - txBufferTail, chunkSize);
            count = MIN(count, DMA_MAX_BURST_DATA_TRANSFER); // MIN(remaining in the buffer, len_truncate, max_burst)
            dmaChannelSend->sourceBuffer(&txBuffer[txBufferTail], count);
            dmaChannelSend->enable();
            __enable_irq();
        }
    }
    return len;

}

void DmaSerialTeensy::rxIsr() {
    dmaChannelReceive->clearInterrupt();

    // no need for rx interrupt
    /*
    // move the head:
    int count = dmaChannelReceive->TCD->BITER;
    rxBufferHead += count;
    if (rxBufferHead >= DMA_RX_BUFFER_SIZE)
        rxBufferHead -= DMA_RX_BUFFER_SIZE;
    */
}

void DmaSerialTeensy::txIsr() {
    dmaChannelSend->clearInterrupt();

    // move the tail:
    {
        int count = dmaChannelSend->TCD->BITER;

        txBufferTail += count;
        if (txBufferTail >= DMA_TX_BUFFER_SIZE)
            txBufferTail -= DMA_TX_BUFFER_SIZE;
        txBufferCount -= count;
    }

    if (txBufferCount > 0) {
        transmitting = true;
        __disable_irq()
        int count = MIN(DMA_TX_BUFFER_SIZE - txBufferTail, txBufferCount);
        count = MIN(count, DMA_MAX_BURST_DATA_TRANSFER); // MIN(remaining in the buffer, txBufferCount, max_burst)
        dmaChannelSend->sourceBuffer(&txBuffer[txBufferTail], count);
        dmaChannelSend->enable();
        __enable_irq();
    }
    else {
        transmitting = false;
    }
}


