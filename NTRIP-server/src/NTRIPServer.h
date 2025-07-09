#ifndef NTRIPSERVER_H
#define NTRIPSERVER_H

#include <WiFiClient.h>

class NTRIPServer {
public:
    NTRIPServer();   // Constructor declaration
    ~NTRIPServer();  // Destructor declaration

    WiFiClient& getClient();
    int available();
    int read();
    bool connected();
    size_t write(const uint8_t* data, size_t len);
    void disconnect();

    // Connect to mountpoint for receiving corrections (caster)
    bool connectToMountpointDirect(const char* host, int port, const char* mountpoint, const char* username, const char* password);

    // Connect as source (if you want to implement sending corrections upstream)
    bool connectToMountpointSource(const char* host, int port, const char* mountpoint, const char* username, const char* password);

private:
    WiFiClient _client;
    bool _connected;
};

#endif


