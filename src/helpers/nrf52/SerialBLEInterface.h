#pragma once

#include "../BaseSerialInterface.h"
#include <bluefruit.h>

#ifndef BLE_TX_POWER
#define BLE_TX_POWER 4
#endif

// BLE serial interface implementation for nRF52 using Bluefruit library
// Provides frame-based communication over BLE UART service with connection management
class SerialBLEInterface : public BaseSerialInterface {
  BLEUart bleuart;
  bool _isEnabled;
  bool _isDeviceConnected;
  volatile uint8_t _pending_writes;
  bool _advRestartPending;
  uint32_t _advRestartTime;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  #define FRAME_QUEUE_SIZE  6   // Application-level frame buffer before sending to BLE
  #define MAX_PENDING_WRITES 12  // Limit concurrent writes (SoftDevice HVN queue is 16, leaving 4 slots headroom)
  #define CONNECT_EVENT_GRACE_PERIOD 3000  // Delay advertising restart for 3s after disconnect to allow iOS/Android to clean up stale connection state
  // iOS connection supervision timeout can be up to 6s, but most are shorter. 3s is safe without delaying legitimate reconnections.
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];

  void clearBuffers() {
    send_queue_len = 0;
    _pending_writes = 0;
  }
  static void onConnect(uint16_t connection_handle);
  static void onDisconnect(uint16_t connection_handle, uint8_t reason);
  static void onSecured(uint16_t connection_handle);
  static bool onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request);
  static void onPairingComplete(uint16_t connection_handle, uint8_t auth_status);
  static void onBLEEvent(ble_evt_t* evt);

public:
  SerialBLEInterface() {
    _isEnabled = false;
    _isDeviceConnected = false;
    send_queue_len = 0;
    _pending_writes = 0;
    _advRestartPending = false;
    _advRestartTime = 0;
  }

  // Start BLE advertising to allow connections
  void startAdv();
  // Stop BLE advertising
  void stopAdv();
  // Initialize BLE stack, configure security, and set up advertising
  void begin(const char* device_name, uint32_t pin_code);
  // Disconnect all active BLE connections
  void disconnect();

  // Enable interface and start advertising
  void enable() override;
  // Disable interface, disconnect, and stop advertising
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  // Check if device is connected and connection handle is valid
  bool isConnected() const override;

  // Check if write queue is at capacity
  bool isWriteBusy() const override;
  // Queue frame for transmission over BLE
  size_t writeFrame(const uint8_t src[], size_t len) override;
  // Process received frames and handle outgoing frame queue
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
