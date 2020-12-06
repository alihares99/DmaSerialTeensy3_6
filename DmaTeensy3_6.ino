
#include "DmaSerialTeensy.h"

void setup() {

    dmaSerial1.begin(115200);
    dmaSerial2.begin(115200);
    dmaSerial3.begin(115200);
    dmaSerial4.begin(115200);
    delay(1000);
}

void loop() {
    static uint32_t last = 1000;
    if (millis() >= last + 1000) {
        last = millis();

        dmaSerial1.println("alive\r\n");
        dmaSerial2.println("alive\r\n");
        dmaSerial3.println("alive\r\n");
        dmaSerial4.println("alive\r\n");

    }

	while (dmaSerial1.available()) {
		dmaSerial1.read();
	}

    while (dmaSerial2.available()) {
        dmaSerial2.write(dmaSerial2.read());
    }
    while (dmaSerial3.available()) {
        dmaSerial3.write(dmaSerial3.read());
    }
    while (dmaSerial4.available()) {
        dmaSerial4.write(dmaSerial4.read());
    }

}
