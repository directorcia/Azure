/*
  WiFiConnect.h - Library for managing WiFi connection for Arduino Uno R4 WiFi
*/

#ifndef WiFiConnect_h
#define WiFiConnect_h

#include <WiFiS3.h>

class WiFiConnect {
  public:
    WiFiConnect(const char* ssid, const char* password);
    bool connect(int timeout = 10000); // Timeout in milliseconds
    bool isConnected();
    IPAddress getIP();
    String getMacAddress();
    int getSignalStrength();
    void disconnect();
    
  private:
    const char* _ssid;
    const char* _password;
};

#endif