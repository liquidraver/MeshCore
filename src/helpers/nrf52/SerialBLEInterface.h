#pragma once

#include "../BaseSerialInterface.h"
#include <bluefruit.h>

#ifndef BLE_TX_POWER
#define BLE_TX_POWER 2
#endif

class SerialBLEInterface : public BaseSerialInterface {
  BLEUart bleuart;
  bool _isEnabled;
  bool _isDeviceConnected;
  uint16_t _connectionHandle;  // Track specific connection handle
  volatile uint8_t _pending_writes;  // Track pending BLE notifications in SoftDevice queue
  bool _advRestartPending;  // Track if advertising restart is scheduled (iPhone crash prevention)
  uint32_t _advRestartTime;  // Time when advertising should restart (iPhone crash prevention)

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  #define FRAME_QUEUE_SIZE  4
  #define MAX_PENDING_WRITES 8  // Bernoulli’s principle - we want to be ready when softdevice queue is drained - TURBO MODE
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];

  void clearBuffers() {
    send_queue_len = 0;
    _pending_writes = 0;
  }
  bool isConnectionHandleValid() const {
    return _connectionHandle != 0xFFFF;
  }
  static void onConnect(uint16_t connection_handle);
  static void onDisconnect(uint16_t connection_handle, uint8_t reason);
  static void onSecured(uint16_t connection_handle);
  static bool onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request);
  static void onPairingComplete(uint16_t connection_handle, uint8_t auth_status);
  static void onBLEEvent(ble_evt_t* evt);  // Hook into SoftDevice events for TX completion

public:
  SerialBLEInterface() {
    _isEnabled = false;
    _isDeviceConnected = false;
    _connectionHandle = 0xFFFF;  // BLE_CONN_HANDLE_INVALID (standard invalid handle value)
    send_queue_len = 0;
    _pending_writes = 0;
    _advRestartPending = false;
    _advRestartTime = 0;
  }

  void startAdv();
  void stopAdv();
  void begin(const char* device_name, uint32_t pin_code);
  void disconnect();  // Disconnect and wait for completion

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;
};

#if BLE_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define BLE_DEBUG_PRINT(F, ...) Serial.printf("BLE: " F, ##__VA_ARGS__)
  #define BLE_DEBUG_PRINTLN(F, ...) Serial.printf("BLE: " F "\n", ##__VA_ARGS__)
#else
  #define BLE_DEBUG_PRINT(...) {}
  #define BLE_DEBUG_PRINTLN(...) {}
#endif
