#pragma once

#include "../BaseSerialInterface.h"
#include <bluefruit.h>

#ifndef BLE_TX_POWER
#define BLE_TX_POWER 4
#endif

class SerialBLEInterface : public BaseSerialInterface {
  BLEUart bleuart;
  bool _isEnabled;
  bool _isDeviceConnected;
  uint16_t _connectionHandle;
  volatile uint8_t _pending_writes;
  bool _advRestartPending;
  uint32_t _advRestartTime;

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  #define FRAME_QUEUE_SIZE  4
  #define MAX_PENDING_WRITES 8
  #define MAX_WRITE_RETRIES 3
  #define MAX_WRITE_FAILURE_DURATION 5000  // Force disconnect if writes fail for 5 seconds
  #define MIN_DISCONNECT_INTERVAL 3000  // Minimum time between disconnects (3 seconds) to prevent rapid cycles
  #define DISCONNECT_EVENT_TIMEOUT 2000  // If disconnect event doesn't arrive within 2 seconds, assume disconnected
  #define TX_QUEUE_DRAIN_TIMEOUT 2000  // Maximum time to wait for TX queue to drain before forcing disconnect
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];
  uint8_t _write_retry_count;
  unsigned long _first_write_failure_time;  // Track when write failures started
  unsigned long _last_disconnect_time;  // Track last disconnect time for rate limiting
  unsigned long _disconnect_initiated_time;  // Track when disconnect was initiated for timeout
  bool _disconnect_pending;  // Track if we're waiting for disconnect event
  bool _disconnect_waiting_tx_drain;  // Track if we're waiting for TX queue to drain before disconnecting
  unsigned long _tx_drain_wait_start;  // Track when we started waiting for TX queue to drain

  void clearBuffers() {
    send_queue_len = 0;
    _pending_writes = 0;
    _write_retry_count = 0;
    _first_write_failure_time = 0;
  }
  bool isConnectionHandleValid() const {
    return _connectionHandle != 0xFFFF;
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
    _connectionHandle = 0xFFFF;
    send_queue_len = 0;
    _pending_writes = 0;
    _advRestartPending = false;
    _advRestartTime = 0;
    _write_retry_count = 0;
    _first_write_failure_time = 0;
    _last_disconnect_time = 0;
    _disconnect_initiated_time = 0;
    _disconnect_pending = false;
    _disconnect_waiting_tx_drain = false;
    _tx_drain_wait_start = 0;
  }

  void startAdv();
  void stopAdv();
  void begin(const char* device_name, uint32_t pin_code);
  void disconnect();

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
