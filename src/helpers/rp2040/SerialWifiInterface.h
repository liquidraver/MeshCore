#pragma once

#include "../BaseSerialInterface.h"
#include <WiFi.h>

#if WIFI_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define WIFI_DEBUG_PRINT(F, ...) Serial.printf("WiFi: " F, ##__VA_ARGS__)
  #define WIFI_DEBUG_PRINTLN(F, ...) Serial.printf("WiFi: " F "\n", ##__VA_ARGS__)
#else
  #define WIFI_DEBUG_PRINT(...) {}
  #define WIFI_DEBUG_PRINTLN(...) {}
#endif

class SerialWifiInterface : public BaseSerialInterface {
  bool _deviceConnected;
  bool _isEnabled;

  WiFiServer server;
  WiFiClient client;

  // Receive staging buffer for frame reassembly
  uint8_t _rx_buf[MAX_FRAME_SIZE + 3];  // header + payload
  uint16_t _rx_buf_len;

  void clearBuffers() {
    _rx_buf_len = 0;
  }

public:
  SerialWifiInterface() : server(WiFiServer()), client(WiFiClient()) {
    _deviceConnected = false;
    _isEnabled = false;
    _rx_buf_len = 0;
  }

  void begin(int port);

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;
  bool isWriteBusy() const override;

  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;
};
