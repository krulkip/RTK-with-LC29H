#include <WiFi.h>
#include <WiFiClient.h>
#include <NTRIPServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TinyGPSPlus.h>
#include "RTCMWrapper.h"
#include <secret.h>

#define DEBUG 1
#define NMEA  1

int rtcm = 0;

/*
// WiFi defined inside secret.h like so
// ---------------- WiFi Credentials ----------------
const char* ssid     = "YOUR SSID";
const char* password = "YOUR PASSWORD";
// Your location defined inside secret.h like so
#define X 1234567.587
#define Y 123456.242
#define Z 1234567.793
// Casters defined inside secret.h like so
// ---------------- RTK2go Caster ----------------
int  caster1_port = 2101;
char caster1_host[] = "RTK2go.com";
char mountpoint1[]  = "MOUNTPOINT1";
char user1[]        = "USER_NAME";
char pass1[]        = "PASSWORD";

// ---------------- Onocoy Caster ----------------
int  caster2_port = 2101;
char caster2_host[] = "servers.onocoy.com";   // <- put your Onocoy host here
char mountpoint2[]  = "MOUNTPOINT2";          // <- Onocoy mountpoint
char user2[]        = "MOUNTPOINT2";          // <- Onocoy username = mountpoint
char pass2[]        = "PASSWORD";              // <- Onocoy password
*/

String inputLine = "";
char srcSTR[] = "";

WiFiClient client1;
NTRIPServer ntrip1;

WiFiClient client2;
NTRIPServer ntrip2;

TinyGPSPlus gps;
RTCMStreamSplitter splitter;

#define MAX_RTCM_LEN 512
uint8_t rtcmBuf[MAX_RTCM_LEN];
char lastGGASentence[128] = {0};

uint8_t peerMac[] = {0xF4, 0x65, 0x0B, 0x4A, 0x84, 0x04};

#define GNSS_RX 16
#define GNSS_TX 17
#define LED_BUILTIN 2

unsigned long reconnectInterval1 = 30000;
unsigned long lastConnect1 = millis() - reconnectInterval1;
int reconnectAttempts1 = 0;
const int maxReconnectAttempts1 = 5;
bool caster1Connected = false;

unsigned long reconnectInterval2 = 30000;
unsigned long lastConnect2 = millis() - reconnectInterval2;
int reconnectAttempts2 = 0;
const int maxReconnectAttempts2 = 5;
bool caster2Connected = false;

bool prevGpsFix = false;
unsigned long lastGnssPrint = 0;
unsigned long gnssPrintInterval = 500;

bool rtcInProgress = false;
bool readingNMEA = false;
char nmeaBuffer[128];
uint8_t nmeaIndex = 0;

bool gpsHasFix = false;
bool firstGGASent = false;

unsigned long lastGgaSend = 0;
const unsigned long ggaSendInterval = 8000;  // 8 seconds

String sendCommandAndGetResponse(Stream &Serial2, const String &command, const String explain, unsigned long timeout = 300) {
    Serial.print(command); Serial.print("  "); Serial.println(explain);
    while (Serial2.available()) Serial2.read();
    Serial2.println(command); delay(10);
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        while (Serial2.available()) {
            char c = Serial2.read();
            response += c;
            if (c == '\n') {
                if (response.indexOf("VERNO") != -1) {
                    response = "[VERSION INFO] " + response;
                }
                Serial.print("LC29H: "); Serial.println(response);
                return response;
            }
        }
    }
    Serial.print("LC29H: "); Serial.println(response);
    return response;
}

bool waitForNTRIPResponse(NTRIPServer &ntrip, unsigned long timeout) {
  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeout) {
    while (ntrip.available()) {
      char c = ntrip.read();
      response += c;

      if (response.endsWith("\r\n")) {
        Serial.print("Caster response: ");
        Serial.print(response);

        if (response.indexOf("ICY 200") >= 0 || 
            response.indexOf("HTTP/1.1 200") >= 0 || 
            response.indexOf("SOURCETABLE 200 OK") >= 0) {
          Serial.println("✅ Caster responded OK");
          return true;
        }

        if (response.indexOf("406") >= 0 || 
            response.indexOf("Not Acceptable") >= 0 || 
            response.indexOf("In Start Up Phase") >= 0) {
          Serial.println("⚠️  Caster is in startup phase, retrying shortly...");
          delay(30000);
          return false;
        }

        if (response.indexOf("401 Unauthorized") >= 0) {
          Serial.println("❌ Unauthorized! Check username/password.");
          return false;
        }

        if (response.indexOf("404") >= 0 || 
            response.indexOf("No such mountpoint") >= 0) {
          Serial.println("❌ Mountpoint not found on caster.");
          return false;
        }

        Serial.println("⚠️  Unrecognized response from caster.");
        return false;
      }
    }
    delay(10);
  }

  Serial.println("⏳ Timeout waiting for caster response.");
  return false;
}

bool isNtrip1Connected() {
  return ntrip1.connected() && caster1Connected;
}
bool isNtrip2Connected() {
  return ntrip2.connected() && caster2Connected;
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(460800, SERIAL_8N1, GNSS_RX, GNSS_TX);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Starting...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    delay(100);
    Serial.print("WiFi.localIP = ");Serial.println(WiFi.localIP());

    client1.setTimeout(10000);
    client2.setTimeout(10000);

    sendCommandAndGetResponse(Serial2, "$PQTMVERNO*58", "Get version number");
    sendCommandAndGetResponse(Serial2, "$PAIR062,0,1*3F", "Turn on GGA message");
    sendCommandAndGetResponse(Serial2, "$PAIR432,1*22",  "Enable RTCM3 MSM7 messages");
    sendCommandAndGetResponse(Serial2, "$PAIR434,1*24",  "Enable RTCM3 antenna location 1005 message");
    sendCommandAndGetResponse(Serial2, "$PAIR436,1*26",  "Enable RTCM3 ephemeris message");
    sendCommandAndGetResponse(Serial2, "$PQTMCFGFIXRATE,R*71", "Read fix rate");
//sendCommandAndGetResponse(Serial2, "$PQTMRESTOREPAR*13", "PQTM parameters to default and reset");
//sendCommandAndGetResponse(Serial2, "$PQTMCFGRCVRMODE,W,2*29","Set in Base mode");
//sendCommandAndGetResponse(Serial2, "$PQTMSAVEPAR*5A","Save settings");
//sendCommandAndGetResponse(Serial2, "$PQTMCFGSVIN,W,2,0,0,X,Y,Z*04", "fix location");
//sendCommandAndGetResponse(Serial2, "$PQTMCFGFIXRATE,W,200*6A", "Set fix rate 200ms ie 5Hz");

//sendCommandAndGetResponse(Serial2, "$PAIR062,1,1*3E","Turn on message 0 GLL");
//sendCommandAndGetResponse(Serial2, "$PAIR062,2,1*3D","Turn on message 0 GSA");
//sendCommandAndGetResponse(Serial2, "$PAIR062,3,1*3C","Turn on message 0 GSV");
//sendCommandAndGetResponse(Serial2, "$PAIR062,4,1*3B","Turn on message 0 RMC");
//sendCommandAndGetResponse(Serial2, "$PAIR062,5,1*3A","Turn on message 0 VTG");
//sendCommandAndGetResponse(Serial2, "$PAIR062,6,1*39","Turn on message 0 ZDA");
//sendCommandAndGetResponse(Serial2, "$PAIR062,7,1*38","Turn on message 0 GRS");
//sendCommandAndGetResponse(Serial2, "$PAIR062,8,1*37","Turn on message 0 GST");

//sendCommandAndGetResponse(Serial2, "$PQTMCFGMSGRATE,W,PQTMEPE,1,2*1D","Enable PQTMEPE");
//sendCommandAndGetResponse(Serial2, "$PAIR411*3E","1005");
//sendCommandAndGetResponse(Serial2, "$PAIR412,1*","1077");
//sendCommandAndGetResponse(Serial2, "$PAIR413,1*","1087");
//sendCommandAndGetResponse(Serial2, "$PAIR417,1*","1230");
//sendCommandAndGetResponse(Serial2, "$PAIR382,1*2E","save ??");
//sendCommandAndGetResponse(Serial2, "$PAIR511*3F","Save current navigation data from RTC RAM to flash");
//sendCommandAndGetResponse(Serial2, "$PAIR513*3D","Save file system");


    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (true);
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer");
    } else {
        Serial.println("Added ESP-NOW peer");
    }
    Serial.print("WiFi channel = "); Serial.println(WiFi.channel());
}

void loop() {
    // Read serial input line and forward to Serial2
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputLine.length() > 0) {
                Serial2.println(inputLine);
                inputLine = "";
            }
        } else {
            inputLine += c;
        }
    }

    // Read from Serial2 and parse RTCM + NMEA
    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        int rtcmMsgType = splitter.inputByte(b);
        if (b == 0xD3) rtcInProgress = true;

        if (rtcmMsgType > 0) {
            rtcInProgress = false;

            if (caster1Connected && firstGGASent) {
                Serial.printf("%d Sending RTCM (%d bytes) to RTK2go caster\n", rtcmMsgType, splitter.outputStreamLength);
                handleRTCM(splitter.outputStream, splitter.outputStreamLength, 1);
            }
            if (caster2Connected && firstGGASent) {
                Serial.printf("%d Sending RTCM (%d bytes) to Onocoy caster\n", rtcmMsgType, splitter.outputStreamLength);
                handleRTCM(splitter.outputStream, splitter.outputStreamLength, 2);
            }
        }

        if (!rtcInProgress) {
            if (b == '$') {
                Serial.println("RTCM data complete");
                readingNMEA = true;
                nmeaIndex = 0;
                nmeaBuffer[nmeaIndex++] = b;
                continue;
            }
            if (readingNMEA) {
                if (b == '\n') {
                    if (nmeaIndex > 0 && nmeaBuffer[nmeaIndex - 1] == '\r') nmeaIndex--;
                    nmeaBuffer[nmeaIndex] = '\0';
                    readingNMEA = false;

                    Serial.print("NMEA: ");
                    Serial.println((char*)nmeaBuffer);

                    if (strncmp((char*)nmeaBuffer, "$GNGGA", 6) == 0) {
                        strncpy(lastGGASentence, nmeaBuffer, sizeof(lastGGASentence) - 1);
                        for (size_t i = 0; i < nmeaIndex; i++) gps.encode(nmeaBuffer[i]);
                        gps.encode('\r'); gps.encode('\n');

                        if (!firstGGASent && (caster1Connected || caster2Connected)) {
                            if (caster1Connected) {
                                ntrip1.write((const uint8_t*)nmeaBuffer, strlen(nmeaBuffer));
                                ntrip1.write((const uint8_t*)"\r\n", 2);
                                Serial.println("📡 Sent first live GGA to RTK2go caster");
                            }
                            if (caster2Connected) {
                                ntrip2.write((const uint8_t*)nmeaBuffer, strlen(nmeaBuffer));
                                ntrip2.write((const uint8_t*)"\r\n", 2);
                                Serial.println("📡 Sent first live GGA to Onocoy caster");
                            }
                            firstGGASent = true;
                        }

                        if (gps.satellites.value() > 4 && gps.hdop.hdop() < 5.0 && gps.location.isValid()) {
                            gpsHasFix = true;
                        } else {
                            gpsHasFix = false;
                        }

                        if (gpsHasFix && !prevGpsFix) Serial.println("✅ GNSS fix acquired");
                        if (!gpsHasFix && prevGpsFix) Serial.println("❌ GNSS fix lost");

                        prevGpsFix = gpsHasFix;
                    }
                    nmeaIndex = 0;
                } else if (nmeaIndex < sizeof(nmeaBuffer) - 1) {
                    nmeaBuffer[nmeaIndex++] = b;
                } else {
                    readingNMEA = false;
                    nmeaIndex = 0;
                }
            }
        }
    }

    // Detect unexpected disconnections
    if (caster1Connected && !ntrip1.connected()) {
        Serial.println("⚠️ RTK2go caster disconnected unexpectedly. Will attempt reconnect.");
        caster1Connected = false;
        lastConnect1 = 0;
    }
    if (caster2Connected && !ntrip2.connected()) {
        Serial.println("⚠️ Onocoy caster disconnected unexpectedly. Will attempt reconnect.");
        caster2Connected = false;
        lastConnect2 = 0;
    }

    unsigned long now = millis();
    if (gpsHasFix && firstGGASent) {
        if (now - lastGgaSend > ggaSendInterval) {
            if (strlen(lastGGASentence) > 0) {
                if (caster1Connected) {
                    ntrip1.write((const uint8_t*)lastGGASentence, strlen(lastGGASentence));
                    ntrip1.write((const uint8_t*)"\r\n", 2);
                    Serial.println("📡 Sent periodic GGA to RTK2go caster");
                }
                if (caster2Connected) {
                    ntrip2.write((const uint8_t*)lastGGASentence, strlen(lastGGASentence));
                    ntrip2.write((const uint8_t*)"\r\n", 2);
                    Serial.println("📡 Sent periodic GGA to Onocoy caster");
                }
                lastGgaSend = now;
            }
        }
    }

    tryReconnect();
}

void handleRTCM(const uint8_t* data, size_t len, int casterNumber) {
    if (casterNumber == 1) {
        if (isNtrip1Connected()) {
            ntrip1.write(data, len);
        } else {
            Serial.println("❗ Tried to send RTCM to RTK2go but NTRIP not connected");
        }
    }
    if (casterNumber == 2) {
        if (isNtrip2Connected()) {
            ntrip2.write(data, len);
        } else {
            Serial.println("❗ Tried to send RTCM to Onocoy but NTRIP not connected");
        }
    }

    static unsigned long lastSendTime = 0;
    const unsigned long sendInterval = 3;
    size_t offset = 0;

    while (offset < len) {
        if (millis() - lastSendTime >= sendInterval) {
            size_t chunkSize = min(250U, len - offset);
            esp_err_t result = esp_now_send(peerMac, data + offset, chunkSize);
            if (result != ESP_OK) {
                Serial.println("ESP-NOW send failed");
            }
            offset += chunkSize;
            lastSendTime = millis();
        }
        yield();
    }

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1);
    digitalWrite(LED_BUILTIN, LOW);
}

void tryReconnect() {
    unsigned long now = millis();

    if (!gpsHasFix) {
        if (now - lastGnssPrint >= gnssPrintInterval) {
            Serial.print(".");
            lastGnssPrint = now;
        }
        return;
    }

    if (gpsHasFix && !prevGpsFix) {
        Serial.println();
        Serial.println("✅ GNSS fix acquired, starting NTRIP reconnect");
        lastConnect1 = 0;
        lastConnect2 = 0;
        reconnectInterval1 = 30000;
        reconnectInterval2 = 30000;
    }
    prevGpsFix = gpsHasFix;

    // RTK2go reconnect logic
    if ((!ntrip1.connected() || !caster1Connected) && (now - lastConnect1 > reconnectInterval1)) {
        if (!client1.connected()) {
            Serial.println("⚠️ RTK2go TCP client not connected, forcing cleanup");
            caster1Connected = false;
            client1.stop();
        }
        if (reconnectAttempts1 < maxReconnectAttempts1) {
            Serial.println("🔄 Attempting RTK2go caster connection...");

            if (ntrip1.connectToMountpointDirect(caster1_host, caster1_port, mountpoint1, user1, pass1)) {
                Serial.println("✅ Connected to RTK2go mountpoint.");
                caster1Connected = true;
                reconnectAttempts1 = 0;
                reconnectInterval1 = 30000;

                if (strlen(lastGGASentence) > 0) {
                    ntrip1.write((const uint8_t*)lastGGASentence, strlen(lastGGASentence));
                    ntrip1.write((const uint8_t*)"\r\n", 2);
                    Serial.println("📡 Sent latest stored GGA after RTK2go connect");
                    firstGGASent = true;
                }
            } else {
                Serial.println("❌ RTK2go reconnect attempt failed");
                reconnectAttempts1++;
                reconnectInterval1 = min((unsigned long)(reconnectInterval1 * 1.5 + random(1000, 3000)), 300000UL);
            }
            lastConnect1 = now;
        } else {
            Serial.println("❌ RTK2go max reconnect attempts reached, backing off...");
            reconnectAttempts1 = 0;
            reconnectInterval1 = 300000;
            lastConnect1 = now;
        }
    }

    // Onocoy reconnect logic
    if ((!ntrip2.connected() || !caster2Connected) && (now - lastConnect2 > reconnectInterval2)) {
        if (!client2.connected()) {
            Serial.println("⚠️ Onocoy TCP client not connected, forcing cleanup");
            caster2Connected = false;
            client2.stop();
        }
        if (reconnectAttempts2 < maxReconnectAttempts2) {
            Serial.println("🔄 Attempting Onocoy caster connection...");

            if (ntrip2.connectToMountpointDirect(caster2_host, caster2_port, mountpoint2, user2, pass2)) {
                Serial.println("✅ Connected to Onocoy mountpoint.");
                caster2Connected = true;
                reconnectAttempts2 = 0;
                reconnectInterval2 = 30000;

                if (strlen(lastGGASentence) > 0) {
                    ntrip2.write((const uint8_t*)lastGGASentence, strlen(lastGGASentence));
                    ntrip2.write((const uint8_t*)"\r\n", 2);
                    Serial.println("📡 Sent latest stored GGA after Onocoy connect");
                    firstGGASent = true;
                }
            } else {
                Serial.println("❌ Onocoy reconnect attempt failed");
                reconnectAttempts2++;
                reconnectInterval2 = min((unsigned long)(reconnectInterval2 * 1.5 + random(1000, 3000)), 300000UL);
            }
            lastConnect2 = now;
        } else {
            Serial.println("❌ Onocoy max reconnect attempts reached, backing off...");
            reconnectAttempts2 = 0;
            reconnectInterval2 = 300000;
            lastConnect2 = now;
        }
    }
}
