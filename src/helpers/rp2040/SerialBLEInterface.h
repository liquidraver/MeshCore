#pragma once

#include "../SerialBLECommon.h"
#include <BTstackLib.h>

// Bonding persistence: see btstack_flash_bank_fs.cpp

class SerialBLEInterface : public SerialBLEInterfaceBase {
  uint32_t _pin_code;
  bool _isAdvertising;
  bool _can_send_now;

  uint16_t _tx_handle;
  uint16_t _rx_handle;
  bool _tx_subscribed;

  char _device_name[32];

  // Must remain valid after gap_scan_response_set_data() / gap_advertisements_set_data()
  uint8_t _scan_rsp_data[31];
  uint8_t _scan_rsp_len;
  uint8_t _adv_data[31];
  uint8_t _adv_data_len;

  static SerialBLEInterface* instance;

  void clearBuffers();
  bool isValidConnection(uint16_t conn_handle, bool requireWaitingForSecurity = false) const;
  bool isAdvertising() const;
  void requestSyncModeConnection();
  void requestDefaultConnection();
  void startAdvertising();
  void stopAdvertising();

  static void deviceConnectedCallback(BLEStatus status, BLEDevice* device);
  static void deviceDisconnectedCallback(BLEDevice* device);
  static int gattWriteCallback(uint16_t att_handle, uint8_t* buffer, uint16_t buffer_size);
  static uint16_t gattReadCallback(uint16_t att_handle, uint8_t* buffer, uint16_t buffer_size);

  friend void btstack_security_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
  friend void btstack_connection_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

public:
  SerialBLEInterface() {
    _pin_code = 0;
    _isAdvertising = false;
    _can_send_now = false;
    _tx_handle = 0;
    _rx_handle = 0;
    _tx_subscribed = false;
    initCommonState();
  }

  void begin(const char* device_name, uint32_t pin_code);
  void disconnect();

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }
  bool isConnected() const override;
  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;
};
