#include "SerialBLEInterface.h"
#include "../SerialBLECommon.h"
#include <string.h>
#include <Arduino.h>

extern "C" {
  #include <btstack.h>
  #include <ble/att_db_util.h>
}

SerialBLEInterface* SerialBLEInterface::instance = nullptr;

// BTstack UUID objects using common definitions
static UUID nus_service_uuid(SERVICE_UUID);
static UUID nus_rx_char_uuid(CHARACTERISTIC_UUID_RX);
static UUID nus_tx_char_uuid(CHARACTERISTIC_UUID_TX);

void btstack_security_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;
  
  if (packet_type != HCI_EVENT_PACKET) return;
  
  uint8_t event = hci_event_packet_get_type(packet);
  
  switch (event) {
    case SM_EVENT_PAIRING_COMPLETE: {
      uint8_t status = sm_event_pairing_complete_get_status(packet);
      hci_con_handle_t conn_handle = sm_event_pairing_complete_get_handle(packet);

      BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing complete handle=0x%04X status=%u", conn_handle, status);

      if (SerialBLEInterface::instance) {
        if (SerialBLEInterface::instance->isValidConnection(conn_handle, true)) {
          if (status == ERROR_CODE_SUCCESS) {
            BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing successful");
            SerialBLEInterface::instance->_isDeviceConnected = true;
            SerialBLEInterface::instance->_can_send_now = true;
            SerialBLEInterface::instance->_last_activity_time = millis();
            SerialBLEInterface::instance->requestSyncModeConnection();
          } else {
            BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing failed, disconnecting");
            SerialBLEInterface::instance->disconnect();
          }
        }
      }
      break;
    }
    
    case SM_EVENT_REENCRYPTION_COMPLETE: {
      uint8_t status = sm_event_reencryption_complete_get_status(packet);
      hci_con_handle_t conn_handle = sm_event_reencryption_complete_get_handle(packet);

      if (SerialBLEInterface::instance) {
        if (SerialBLEInterface::instance->isValidConnection(conn_handle, true)) {
          if (status == ERROR_CODE_SUCCESS) {
            BLE_DEBUG_PRINTLN("SerialBLEInterface: re-encryption successful");
            SerialBLEInterface::instance->_isDeviceConnected = true;
            SerialBLEInterface::instance->_can_send_now = true;
            SerialBLEInterface::instance->_last_activity_time = millis();
            SerialBLEInterface::instance->requestSyncModeConnection();
          } else {
            BLE_DEBUG_PRINTLN("SerialBLEInterface: re-encryption failed status=%u", status);
          }
        }
      }
      break;
    }
    
    default:
      break;
  }
}

void btstack_connection_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
  (void)channel;
  (void)size;
  
  if (packet_type != HCI_EVENT_PACKET) return;

  uint8_t event = hci_event_packet_get_type(packet);

  if (event == HCI_EVENT_LE_META) {
    uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
    
    if (subevent == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE) {
      hci_con_handle_t conn_handle = hci_subevent_le_connection_update_complete_get_connection_handle(packet);
      uint16_t conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
      uint16_t conn_latency = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
      uint16_t supervision_timeout = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
      
      BLE_DEBUG_PRINTLN("CONN_PARAM_UPDATE: handle=0x%04X, interval=%u, latency=%u, timeout=%u",
                        conn_handle, conn_interval, conn_latency, supervision_timeout);
      
      if (SerialBLEInterface::instance) {
        if (SerialBLEInterface::instance->isValidConnection(conn_handle, false)) {
          SerialBLEInterface::instance->_conn_param_update_pending = false;

          if (conn_latency == BLE_SYNC_SLAVE_LATENCY &&
              supervision_timeout == BLE_SYNC_CONN_SUP_TIMEOUT &&
              conn_interval >= BLE_SYNC_MIN_CONN_INTERVAL &&
              conn_interval <= BLE_SYNC_MAX_CONN_INTERVAL) {
            if (!SerialBLEInterface::instance->_sync_mode) {
              BLE_DEBUG_PRINTLN("Sync mode confirmed by connection parameters");
              SerialBLEInterface::instance->_sync_mode = true;
              SerialBLEInterface::instance->_last_activity_time = millis();
            }
          } else if (conn_latency == BLE_SLAVE_LATENCY &&
                     supervision_timeout == BLE_CONN_SUP_TIMEOUT &&
                     conn_interval >= BLE_MIN_CONN_INTERVAL &&
                     conn_interval <= BLE_MAX_CONN_INTERVAL) {
            if (SerialBLEInterface::instance->_sync_mode) {
              BLE_DEBUG_PRINTLN("Default mode confirmed by connection parameters");
              SerialBLEInterface::instance->_sync_mode = false;
            }
          }
        }
      }
    }
  }
}

static btstack_packet_callback_registration_t sm_event_callback_registration = {
  .callback = &btstack_security_packet_handler
};

static btstack_packet_callback_registration_t connection_event_callback_registration = {
  .callback = &btstack_connection_packet_handler
};

void SerialBLEInterface::deviceConnectedCallback(BLEStatus status, BLEDevice* device) {
  if (!instance) return;

  if (status == BLE_STATUS_OK) {
    uint16_t new_handle = device->getHandle();
    BLE_DEBUG_PRINTLN("SerialBLEInterface: connected handle=0x%04X", new_handle);

    // Reject multiple connections
    if (instance->_conn_handle != BLE_CONN_HANDLE_INVALID && instance->_conn_handle != new_handle) {
      BLE_DEBUG_PRINTLN("SerialBLEInterface: rejecting second connection");
      gap_disconnect(new_handle);
      return;
    }

    instance->_conn_handle = new_handle;
    instance->_isDeviceConnected = false;
    instance->_sync_mode = false;
    instance->_conn_param_update_pending = false;
    instance->_isAdvertising = false;
    instance->_can_send_now = false;
    instance->clearBuffers();
  }
}

void SerialBLEInterface::deviceDisconnectedCallback(BLEDevice* device) {
  if (!instance) return;

  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected handle=0x%04X", device->getHandle());

  if (instance->_conn_handle == device->getHandle()) {
    instance->_conn_handle = BLE_CONN_HANDLE_INVALID;
    instance->_isDeviceConnected = false;
    instance->_sync_mode = false;
    instance->_conn_param_update_pending = false;
    instance->_can_send_now = false;
    instance->_tx_subscribed = false;
    instance->clearBuffers();
    instance->_last_health_check = millis();

    if (instance->_isEnabled) {
      instance->startAdvertising();
    }
  }
}

int SerialBLEInterface::gattWriteCallback(uint16_t att_handle, uint8_t* buffer, uint16_t buffer_size) {
  if (!instance) return 0;
  if (buffer_size == 0) return 0;

  // TX CCCD (handle = TX value handle + 1)
  uint16_t tx_cccd_handle = instance->_tx_handle + 1;
  if (att_handle == tx_cccd_handle && buffer_size == 2) {
    uint16_t cccd_value = buffer[0] | (buffer[1] << 8);
    instance->_tx_subscribed = (cccd_value & 0x0001) != 0;
    return 0;
  }

  // RX characteristic (data from phone)
  if (att_handle == instance->_rx_handle) {
    if (instance->_conn_handle == BLE_CONN_HANDLE_INVALID || !instance->isConnected()) {
      return 0;
    }

    if (buffer_size > MAX_FRAME_SIZE) {
      BLE_DEBUG_PRINTLN("RX: frame too large, len=%u", (unsigned)buffer_size);
      return 0;
    }

    if (instance->recv_queue.isFull()) {
      BLE_DEBUG_PRINTLN("RX: queue full, dropping");
      return 0;
    }

    SerialBLEFrame* frame = instance->recv_queue.getWriteSlot();
    if (frame) {
      frame->len = buffer_size;
      memcpy(frame->buf, buffer, buffer_size);
      instance->recv_queue.push();

      unsigned long now = millis();
      if (instance->noteFrameActivity(now, buffer_size)) {
        instance->requestSyncModeConnection();
      }
    }
    return 0;
  }

  return 0;
}

uint16_t SerialBLEInterface::gattReadCallback(uint16_t characteristic_id, uint8_t* buffer, uint16_t buffer_size) {
  (void)characteristic_id;
  (void)buffer;
  (void)buffer_size;
  return 0;
}

void SerialBLEInterface::clearBuffers() {
  clearTransferState();
  _can_send_now = false;
}

bool SerialBLEInterface::isValidConnection(uint16_t conn_handle, bool requireWaitingForSecurity) const {
  if (_conn_handle != conn_handle) {
    return false;
  }
  if (_conn_handle == BLE_CONN_HANDLE_INVALID) {
    return false;
  }
  if (requireWaitingForSecurity && _isDeviceConnected) {
    return false;
  }
  return true;
}

bool SerialBLEInterface::isAdvertising() const {
  return _isAdvertising;
}

void SerialBLEInterface::startAdvertising() {
  if (_isAdvertising) return;

  // Adv data: Flags (3) + UUID (18) + name (remaining)
  _adv_data_len = 0;

  // Flags: LE General Discoverable, BR/EDR Not Supported
  _adv_data[_adv_data_len++] = 2;
  _adv_data[_adv_data_len++] = 0x01;
  _adv_data[_adv_data_len++] = 0x06;

  // SERVICE_UUID in little-endian for advertising
  static const uint8_t nus_uuid_le[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
  };
  _adv_data[_adv_data_len++] = 17;
  _adv_data[_adv_data_len++] = 0x07;
  memcpy(&_adv_data[_adv_data_len], nus_uuid_le, 16);
  _adv_data_len += 16;

  // Add device name (truncated to fit)
  int remaining = 31 - _adv_data_len - 2;
  if (remaining > 0) {
    size_t name_len = strlen(_device_name);
    int adv_name_len = (remaining < (int)name_len) ? remaining : (int)name_len;
    _adv_data[_adv_data_len++] = adv_name_len + 1;
    _adv_data[_adv_data_len++] = (adv_name_len < (int)name_len) ? 0x08 : 0x09;
    memcpy(&_adv_data[_adv_data_len], _device_name, adv_name_len);
    _adv_data_len += adv_name_len;
  }

  // Set advertising parameters: interval, type, channels
  bd_addr_t null_addr;
  memset(null_addr, 0, 6);
  gap_advertisements_set_params(BLE_ADV_INTERVAL_MIN, BLE_ADV_INTERVAL_MAX, 0, 0, null_addr, 0x07, 0x00);

  BTstack.setAdvData(_adv_data_len, _adv_data);
  BTstack.setScanData(_scan_rsp_len, _scan_rsp_data);
  BTstack.startAdvertising();
  _isAdvertising = true;

  for (int i = 0; i < 10; i++) {
    BTstack.loop();
    delay(10);
  }
}

void SerialBLEInterface::stopAdvertising() {
  if (!_isAdvertising) return;
  BTstack.stopAdvertising();
  _isAdvertising = false;
}

void SerialBLEInterface::requestSyncModeConnection() {
  if (!isConnected()) return;
  if (_sync_mode && !_conn_param_update_pending) return;
  if (_conn_param_update_pending) return;

  BLE_DEBUG_PRINTLN("Requesting sync mode connection: %u-%ums interval, latency=%u, %ums timeout",
    (unsigned)(BLE_SYNC_MIN_CONN_INTERVAL * 125 / 100),
    (unsigned)(BLE_SYNC_MAX_CONN_INTERVAL * 125 / 100),
    BLE_SYNC_SLAVE_LATENCY,
    (unsigned)(BLE_SYNC_CONN_SUP_TIMEOUT * 10));

  int result = gap_request_connection_parameter_update(
    _conn_handle,
    BLE_SYNC_MIN_CONN_INTERVAL,
    BLE_SYNC_MAX_CONN_INTERVAL,
    BLE_SYNC_SLAVE_LATENCY,
    BLE_SYNC_CONN_SUP_TIMEOUT
  );
  if (result == ERROR_CODE_SUCCESS) {
    _conn_param_update_pending = true;
    // Don't set _sync_mode here - wait for HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE callback
  } else {
    BLE_DEBUG_PRINTLN("Failed to request sync mode connection: %d", result);
  }
}

void SerialBLEInterface::requestDefaultConnection() {
  if (!isConnected()) return;
  if (!_sync_mode) return;
  if (!send_queue.isEmpty() || !recv_queue.isEmpty()) return;
  if (_conn_param_update_pending) return;

  BLE_DEBUG_PRINTLN("Requesting default connection: %u-%ums interval, latency=%u, %ums timeout",
    (unsigned)(BLE_MIN_CONN_INTERVAL * 125 / 100),
    (unsigned)(BLE_MAX_CONN_INTERVAL * 125 / 100),
    BLE_SLAVE_LATENCY,
    (unsigned)(BLE_CONN_SUP_TIMEOUT * 10));

  int result = gap_request_connection_parameter_update(
    _conn_handle,
    BLE_MIN_CONN_INTERVAL,
    BLE_MAX_CONN_INTERVAL,
    BLE_SLAVE_LATENCY,
    BLE_CONN_SUP_TIMEOUT
  );
  if (result == ERROR_CODE_SUCCESS) {
    _conn_param_update_pending = true;
    // Don't set _sync_mode here - wait for HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE callback
  } else {
    BLE_DEBUG_PRINTLN("Failed to request default connection: %d", result);
  }
}

// ============================================================================
// Public API
// ============================================================================

void SerialBLEInterface::begin(const char* device_name, uint32_t pin_code) {
  instance = this;
  _pin_code = pin_code;

  strncpy(_device_name, device_name, sizeof(_device_name) - 1);
  _device_name[sizeof(_device_name) - 1] = '\0';

  // GAP Service with Device Name (must be before BTstack.setup())
  att_db_util_add_service_uuid16(0x1800);
  att_db_util_add_characteristic_uuid16(
    0x2A00, ATT_PROPERTY_READ, ATT_SECURITY_NONE, ATT_SECURITY_NONE,
    (uint8_t*)_device_name, strlen(_device_name)
  );

  BTstack.setup(device_name);
  gap_set_local_name(device_name);

  // Set max LE MTU to match ESP32/nRF52
  l2cap_set_max_le_mtu(BLE_MAX_MTU);

  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);

  // Security: PIN + MITM + bonding
  sm_set_io_capabilities(IO_CAPABILITY_DISPLAY_ONLY);
  sm_set_authentication_requirements(SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING);
  sm_use_fixed_passkey_in_display_role(pin_code);
  sm_set_encryption_key_size_range(7, 16);
  sm_add_event_handler(&sm_event_callback_registration);
  hci_add_event_handler(&connection_event_callback_registration);

  BTstack.addGATTService(&nus_service_uuid);

  // Scan response with full device name
  _scan_rsp_len = 0;
  size_t scan_rsp_name_len = strlen(device_name);
  if (scan_rsp_name_len > 29) scan_rsp_name_len = 29;
  _scan_rsp_data[_scan_rsp_len++] = scan_rsp_name_len + 1;
  _scan_rsp_data[_scan_rsp_len++] = 0x09;
  memcpy(&_scan_rsp_data[_scan_rsp_len], device_name, scan_rsp_name_len);
  _scan_rsp_len += scan_rsp_name_len;
  BTstack.setScanData(_scan_rsp_len, _scan_rsp_data);

  // TX characteristic (notify, authenticated)
  _tx_handle = att_db_util_add_characteristic_uuid128(
    (uint8_t*)nus_tx_char_uuid.getUuid(),
    ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY | ATT_PROPERTY_DYNAMIC,
    ATT_SECURITY_AUTHENTICATED, ATT_SECURITY_AUTHENTICATED, NULL, 0
  );

  // RX characteristic (write, authenticated)
  _rx_handle = att_db_util_add_characteristic_uuid128(
    (uint8_t*)nus_rx_char_uuid.getUuid(),
    ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC,
    ATT_SECURITY_NONE, ATT_SECURITY_AUTHENTICATED, NULL, 0
  );

  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);

  for (int i = 0; i < 5; i++) {
    BTstack.loop();
    delay(10);
  }
}

void SerialBLEInterface::disconnect() {
  if (_conn_handle != BLE_CONN_HANDLE_INVALID) {
    gap_disconnect(_conn_handle);
  }
}

void SerialBLEInterface::enable() {
  if (_isEnabled) return;

  _isEnabled = true;
  clearBuffers();
  _last_health_check = millis();

  startAdvertising();
  BTstack.loop();
}

void SerialBLEInterface::disable() {
  _isEnabled = false;
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disable");

  disconnect();
  stopAdvertising();
  _last_health_check = 0;
}

bool SerialBLEInterface::isConnected() const {
  return _isDeviceConnected && _conn_handle != BLE_CONN_HANDLE_INVALID;
}

bool SerialBLEInterface::isWriteBusy() const {
  return isWriteBusyCommon();
}

size_t SerialBLEInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("writeFrame: frame too big, len=%u", (unsigned)len);
    return 0;
  }

  if (!isConnected() || len == 0) {
    return 0;
  }

  if (send_queue.isFull()) {
    BLE_DEBUG_PRINTLN("writeFrame: send_queue is full!");
    return 0;
  }

  SerialBLEFrame* frame = send_queue.getWriteSlot();
  if (frame) {
    frame->len = len;
    memcpy(frame->buf, src, len);
    send_queue.push();
    return len;
  }
  return 0;
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[]) {
  BTstack.loop();

  if (!send_queue.isEmpty()) {
    if (!isConnected()) {
      BLE_DEBUG_PRINTLN("writeBytes: connection invalid, clearing send queue");
      send_queue.init();
    } else if (_can_send_now && _tx_subscribed) {
      unsigned long now = millis();
      bool throttle_active = (_last_retry_attempt > 0 && (now - _last_retry_attempt) < BLE_RETRY_THROTTLE_MS);
      bool send_interval_ok = (_last_send_time == 0 || (now - _last_send_time) >= BLE_MIN_SEND_INTERVAL_MS);

      if (!throttle_active && send_interval_ok) {
        SerialBLEFrame* frame_to_send = send_queue.peekFront();
        if (frame_to_send) {
          bool can_send_now = (_conn_handle != BLE_CONN_HANDLE_INVALID &&
                               att_server_can_send_packet_now(_conn_handle));

          if (can_send_now) {
            int result = att_server_notify(_conn_handle, _tx_handle, frame_to_send->buf, frame_to_send->len);

            if (result == ERROR_CODE_SUCCESS) {
              BLE_DEBUG_PRINTLN("writeBytes: sz=%u, hdr=%u", (unsigned)frame_to_send->len, (unsigned)frame_to_send->buf[0]);
              _last_retry_attempt = 0;
              _last_send_time = now;
              if (noteFrameActivity(now, frame_to_send->len)) {
                requestSyncModeConnection();
              }
              popSendQueue();
            } else {
              BLE_DEBUG_PRINTLN("writeBytes failed (buffer full), keeping frame for retry");
              _last_retry_attempt = now;
            }
          } else {
            if (_conn_handle != BLE_CONN_HANDLE_INVALID) {
              att_server_request_can_send_now_event(_conn_handle);
            }
          }
        }
      }
    }
  }

  if (!recv_queue.isEmpty()) {
    SerialBLEFrame* frame = recv_queue.peekFront();
    if (frame) {
      size_t len = frame->len;
      memcpy(dest, frame->buf, len);
      BLE_DEBUG_PRINTLN("readBytes: sz=%u, hdr=%u", (unsigned)len, (unsigned)dest[0]);
      popRecvQueue();
      return len;
    }
  }

  unsigned long now = millis();
  if (isConnected() && _sync_mode && _last_activity_time > 0 &&
      send_queue.isEmpty() && recv_queue.isEmpty()) {
    if (now - _last_activity_time >= BLE_SYNC_INACTIVITY_TIMEOUT_MS) {
      requestDefaultConnection();
    }
  }

  if (_isEnabled && !isConnected() && _conn_handle == BLE_CONN_HANDLE_INVALID) {
    if (now - _last_health_check >= BLE_HEALTH_CHECK_INTERVAL) {
      _last_health_check = now;

      if (!isAdvertising()) {
        BLE_DEBUG_PRINTLN("SerialBLEInterface: advertising watchdog - restarting");
        startAdvertising();
      }
    }
  }

  return 0;
}
