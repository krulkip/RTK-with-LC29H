#include "RTCMStreamSplitter.h"

class RTCMSplitterWrapper {
public:
    RTCMSplitterWrapper() : readingNMEA(false), nmeaIndex(0) {
        splitter.reset();
    }

    int feedByte(uint8_t b) {
        // First, feed to RTCM splitter
        int msgType = splitter.inputByte(b);
        if (msgType > 0) {
            // Complete RTCM message ready
            // You can get pointer + length from splitter:
            const uint8_t* msg = splitter.buffer();
            size_t len = splitter.length();

            // Do something with RTCM message here or notify caller
            // e.g., print message type:
            Serial.print("RTCM msg type: ");
            Serial.println(msgType);

            splitter.reset(); // ready for next message
            return msgType;
        }

        // If not RTCM, try NMEA
        if (b == '$') {
            readingNMEA = true;
            nmeaIndex = 0;
            nmeaBuffer[nmeaIndex++] = b;
        } else if (readingNMEA) {
            if (b == '\n' || nmeaIndex >= sizeof(nmeaBuffer) - 1) {
                nmeaBuffer[nmeaIndex++] = b;
                nmeaBuffer[nmeaIndex] = '\0';
                readingNMEA = false;
                
                Serial.print("NMEA sentence: ");
                Serial.println((char*)nmeaBuffer);

                // Optionally feed this to TinyGPS++ or other parser here

                nmeaIndex = 0;
            } else {
                nmeaBuffer[nmeaIndex++] = b;
            }
        }
        return 0; // No RTCM message yet
    }

private:
    RTCMStreamSplitter splitter;

    bool readingNMEA;
    uint8_t nmeaBuffer[128];
    size_t nmeaIndex;
};