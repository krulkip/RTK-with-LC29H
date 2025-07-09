#include "NTRIPServer.h"
#include <base64.h>

#define DEBUG 1

NTRIPServer::NTRIPServer() {
  _connected = false;
}

NTRIPServer::~NTRIPServer() {
  disconnect();
}

WiFiClient& NTRIPServer::getClient() {
  return _client;
}

int NTRIPServer::available() {
  return _client.available();
}

int NTRIPServer::read() {
  return _client.read();
}

bool NTRIPServer::connected() {
  return _connected;
}

size_t NTRIPServer::write(const uint8_t* data, size_t len) {
  if (!_connected) return 0;
  return _client.write(data, len);
}

void NTRIPServer::disconnect() {
  if (_client) {
    _client.stop();
  }
  _connected = false;
}

// Connect to NTRIP caster mountpoint directly with debug info and HTTP status check
bool NTRIPServer::connectToMountpointDirect(const char* host, int port, const char* mountpoint, const char* username, const char* password) {
  Serial.printf("🔌 Connecting to NTRIP caster %s:%d...\n", host, port);

  if (_client.connect(host, port)) {
    // Create Basic Auth header from username:password
    //String auth = String(username) + ":" + String(password);
    //String authBase64 = base64::encode(auth);
    String request = "";
    request.reserve(256);  // avoid memory fragmentation
    // Build NTRIP SOURCE request to push RTCM corrections
    if (String(host).indexOf("RTK2go") >= 0) {
       // NTRIP v1.0 request (e.g. RTK2go)
       request += "SOURCE " + String(password) + " /" + String(mountpoint) + "\r\n";
       request += "Source-Agent: NTRIP ESP32\r\n";
       request += "\r\n";
    } else {
       // NTRIP v2.0 request (e.g., Onocoy)
       String credentials = String(username) + ":" + String(password);
       credentials.trim();  // remove any leading/trailing spaces
       String encoded = base64::encode(credentials);
       request += "POST /" + String(mountpoint) + " HTTP/1.1\r\n";
       request += "Host: " + String(host) + "\r\n";
       request += "User-Agent: NTRIP ESP32\r\n";
       request += "Authorization: Basic " + encoded + "\r\n";
       //request += "Ntrip-Version: Ntrip/2.0\r\n";
       request += "\r\n";  // <-- must end with CRLF CRLF  

        //request += "POST /DulyImmenseWarthog HTTP/1.1\r\n";
        //request += "Host: servers.onocoy.com\r\n";
        //request += "User-Agent: NTRIP ESP32\r\n";
        //request += "Authorization: Basic RHVseUltbWVuc2VXYXJ0aG9nOlJUS3Rlc3QzNA==\r\n";
        //request += "Ntrip-Version: Ntrip/2.0\r\n";
        //request += "\r\n";
    }

    Serial.printf("Mountpoint: %s at %s:%d\n", mountpoint, host, port);
    Serial.println("----- NTRIP Request -----");
    Serial.print(request);
    Serial.println("-------------------------");

    _client.print(request);
    _client.flush();  // ✅ Ensure full request is sent before waiting

    // Wait up to 30 seconds for caster response
    Serial.println("⏳ Waiting for caster response (timeout: 30s)...");
    unsigned long timeout = millis() + 30000;
    while (!_client.available() && millis() < timeout) {
      delay(10);
    }

    if (!_client.available()) {
      Serial.println("❌ No response from caster — connection failed");
      _client.stop();
      return false;
    }

    // Read the status line (first line of HTTP response)
    String statusLine = _client.readStringUntil('\n');
    statusLine.trim();
    Serial.print("📡 NTRIP status: ");
    Serial.println(statusLine);

    // Read and print remaining HTTP headers for debugging
    Serial.println("📡 Full response headers:");
    while (_client.available()) {
      String line = _client.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) break;  // Empty line signals end of headers
      Serial.println(line);
    }

    // Check HTTP status and ICY status codes from status line
    if (statusLine.startsWith("ICY 200") || 
        statusLine.startsWith("HTTP/1.1 200") ||
        statusLine.startsWith("HTTP/1.0 200") ||
        statusLine.startsWith("SOURCETABLE")) {
      Serial.println("✅ Connection accepted by caster");
    } 
    else if (statusLine.indexOf("401") >= 0) {
      Serial.println("❌ Unauthorized (401) — check credentials");
      _client.stop();
      return false;
    }
    else if (statusLine.indexOf("403") >= 0) {
      Serial.println("❌ Forbidden (403) — possible IP block or mountpoint permissions issue");
      _client.stop();
      return false;
    }
    else if (statusLine.indexOf("404") >= 0) {
      Serial.println("❌ Not Found (404) — mountpoint does not exist");
      _client.stop();
      return false;
    }
    else if (statusLine.indexOf("406") >= 0) {
      Serial.println("❌ Not Acceptable (406) — caster in startup phase, try again later");
      _client.stop();
      return false;
    }
    else {
      Serial.println("❌ Unexpected caster response — closing connection");
      _client.stop();
      return false;
    }

    // Mark as connected
    _connected = true;
    return true;
  } else {
    Serial.println("❌ TCP connection to caster failed (check DNS, port, firewall)");
    return false;
  }
}