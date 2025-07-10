/*
 *  NTRIP client for Arduino Ver. 1.0.0 
 *  NTRIPClient Sample
 *  Request Source Table (Source Table is basestation list in NTRIP Caster)
 *  Request Reference Data 
 * 
 * 
 */
#include <WiFi.h>           //Need for ESP32 
#include "NTRIPClient.h"
#include "rtcmstreamsplitter.h"
#include <TinyGPSPlus.h>
#include "secret.h"

const char* ssid     = SSID;
const char* password = PASSWORD;

int counter = 0;
#define RX_PIN 16
#define TX_PIN 17
unsigned long lastDisplay = 0;
int rtcm = 0;
int pair = 0;
const unsigned long displayInterval = 1000; // Display every 1000 ms (1 second)
const char* host = HOST;
int httpPort = 2101; //port 2101 is default port of NTRIP caster
const char* mntpnt = MNTPNT;
const char* user   = USER;
const char* passwd = PASSWD;

NTRIPClient ntrip_c;
// The TinyGPSPlus object
TinyGPSPlus gps;
HardwareSerial rtkSerial(2);
RTCMStreamSplitter test;
TinyGPSCustom fixquality(gps, "GNGGA", 6); // $GNGGA sentence, 6th elem
TinyGPSCustom geepees(gps, "PAIR001",1);
int rtcmopt[] ={1005,1127,1046,1020,1033,1012,1019,1004,1087,1230};
int rtcmcnt[]={   0,   0,   0,   0,   0,    0,   0,   0,   0,   0};
String qualityfix[] = {"NOTValid", "GPS fix", "DGPS fix", "XXX", "RTK fix", "Float fix", "Dead Reckoning"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(10);
  rtkSerial.begin(460800,SERIAL_8N1, RX_PIN, TX_PIN);
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Requesting SourceTable.");
  if(ntrip_c.reqSrcTbl(host,httpPort)){
    char buffer[512];
    delay(5);
    while(ntrip_c.available()){
      ntrip_c.readLine(buffer,sizeof(buffer));
      Serial.print(buffer); 
    }
  }
  else{
    Serial.println("SourceTable request error");
  }
  Serial.print("Requesting SourceTable is OK\n");
  ntrip_c.stop(); //Need to call "stop" function for next request.
  
  Serial.println("Requesting MountPoint's Raw data");
  if(!ntrip_c.reqRaw(host,httpPort,mntpnt,user,passwd)){
    delay(15000);
    ESP.restart();
  }
  Serial.println("Requesting MountPoint is OK");
  Serial.println("Configuration Finished");

  RTCMStreamSplitter();
}

void loop() {
  while(ntrip_c.available()) {
        char ch = ntrip_c.read();       
        rtkSerial.print(ch);
        rtcm = test.inputByte(ch);
        if (rtcm==1005) {
           Serial.print(rtcm);Serial.print(" ");
           for (int i=0;i<test.outputStreamLength;i++){
            if (test.outputStream[i]<0x10) {Serial.print("0");}
            Serial.print(test.outputStream[i],HEX);Serial.print(" ");
           }
           Serial.println();   
           int64_t ECEFz = 0, ECEFy = 0, ECEFx = 0;
           for (int i=0;i<5;i++){
              ECEFx = (ECEFx<<8) | test.outputStream[7+i];ECEFy = (ECEFy<<8) | test.outputStream[12+i];ECEFz = (ECEFz<<8) | test.outputStream[17+i];
           }
           
           ECEFy &= 0xFFFFFF3FFFFFFFFFULL;// Make negative number OK
           
           // Example ECEF coordinates (meters)
           double lat, lon, alt;
           double ECEFxScaled = ECEFx*0.0001, ECEFyScaled = ECEFy*0.0001, ECEFzScaled = ECEFz*0.0001;
           ecefToGeodetic(ECEFxScaled, ECEFyScaled , ECEFzScaled, lat, lon, alt);
           Serial.print("Position of NTRIP Caster Latitude: "); Serial.print(lat, 8);Serial.print(" ");
           Serial.print("Longitude: "); Serial.print(lon, 8);Serial.print(" ");
           Serial.print("Altitude: "); Serial.println(alt, 3); // in meters
           Serial.print("Distance to NTRIP Caster = ");
           unsigned long distanceToNTRIP =
           (unsigned long)TinyGPSPlus::distanceBetween(
           gps.location.lat(),
           gps.location.lng(),
           lat, 
           lon);
           printInt(distanceToNTRIP, gps.location.isValid(), 6);
           Serial.println("meters");         
        }
        for (int i=0;i<(sizeof(rtcmopt) / sizeof(rtcmopt[0]));i++){
          if (rtcm == rtcmopt[i]) {rtcmcnt[i]++;}
        }
  }
  while (rtkSerial.available()) {
        //char ch = (char)rtkSerial.read();
        //Serial.print(ch);
        gps.encode(rtkSerial.read());
  }
  if (Serial.available() > 0) {
    String str = Serial.readString();
    str.trim();
    Serial.println(str);
  }
  pair = atoi (geepees.value());
  unsigned long now = millis();
  if (now - lastDisplay >= displayInterval) {
    lastDisplay = now;
    //rtkSerial.println("$PAIR062,0,1*3F");
    if (gps.location.isValid()) {
      Serial.print("Lat: "); Serial.print(gps.location.lat(), 8);
      Serial.print(" | Lng: "); Serial.print(gps.location.lng(), 8);
      Serial.print(" | Height: "); Serial.print(gps.altitude.meters(), 8);
      Serial.print(" | FIX Quality = "); Serial.print(qualityfix[atoi (fixquality.value())]);
      Serial.print(" | $PAIR = "); Serial.print(pair);
      Serial.print(" | RTCM: ");
      for (int i=0;i<(sizeof(rtcmopt) / sizeof(rtcmopt[0]));i++){
        Serial.print(rtcmopt[i]);Serial.print(" ");
        Serial.print(rtcmcnt[i]);Serial.print("; ");
      }
      Serial.println();
    } else {
      Serial.println("Waiting for valid GPS fix...");
    }
  }
}


void ecefToGeodetic(double x, double y, double z, double &lat, double &lon, double &alt) {
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = f * (2 - f);

    lon = atan2(y, x);

    double r = sqrt(x*x + y*y);
    double lat_old = atan2(z, r); // initial guess
    double lat_new = 0;
    double N = 0;
    double alt_temp = 0;
    const double tol = 1e-12; // convergence tolerance

    do {
        lat_new = lat_old;
        N = a / sqrt(1 - e2 * sin(lat_new) * sin(lat_new));
        alt_temp = r / cos(lat_new) - N;
        lat_old = atan2(z, r * (1 - e2 * N / (N + alt_temp)));
    } while (fabs(lat_old - lat_new) > tol);

    lat = lat_old;
    alt = alt_temp;

    lat *= 180.0 / PI;
    lon *= 180.0 / PI;
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (rtkSerial.available())
      gps.encode(rtkSerial.read());
  } while (millis() - start < ms);
}
