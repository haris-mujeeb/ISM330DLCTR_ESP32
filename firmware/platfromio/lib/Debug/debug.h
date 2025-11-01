#ifndef DEBUG_h
#define DEBUG_h

#include "Arduino.h"


/*******************************************************************************
* Debug functions
*******************************************************************************/
//To print string and values
inline void debugPrint(const String &message, const String &value = "") {
    Serial.print(message);
    if (value != "") {
        Serial.print(value);
    }
    Serial.println(); // This will add a new line after the message
}

inline void printBufferHex(const uint8_t* buffer, size_t bufferSize) {
    Serial.println("Buffer Content as Hex:");
    for (size_t i = 0; i < bufferSize; i++) {
        Serial.print("0x");
        // Ensuring hexadecimal numbers are printed as two digits
        if(buffer[i] < 16) {
            Serial.print('0');
        }
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println(); // New line after printing all buffer elements
}

inline void checkIntervalAndPrint(unsigned long &lastMillis, unsigned long interval, const char* text, int &counter) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastMillis >= interval) {
        Serial.print(text);
        Serial.print(counter);
        Serial.println(" times in the last second");

        // Reset the counter and the timer
        counter = 0;
        lastMillis = currentMillis;
    }
}


#endif