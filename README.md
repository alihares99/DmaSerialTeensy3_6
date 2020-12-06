# DMA Serial Library for Teensy 3.6

This small library provides you with the same API as Arduino UART serials like Serial1 but uses DMA and interrupt
in background to improve efficiency. This library is developed for teensy 3.6 but it can ba generalized to support
all teensy 3.x board with a little effort.

In order to use it, put the source codes beside your own project and include the header file.
Then you only need to replace your old Serial1 to 4 with dmaSerial1 to 4 and everything works fine. 
For an example on how to use this library, please refer to DmaTeensy4_0.ino file.

You can change the size of the background buffers by changing the values defined as DMA_TX_BUFFER_SIZE and 
DMA_RX_BUFFER_SIZE.

You are free to use this library in any project as long as you give me credit by keeping my email address
at the top of the header and source file.