#include "SerialBLEInterface.h"
#include <string.h> // For memcpy

static SerialBLEInterface* instance;

void SerialBLEInterface::onConnect(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: connected");
  if (instance) {
    instance->_connectionHandle = connection_handle;  // Store connection handle
    instance->_isDeviceConnected = false;
    instance->_pending_writes = 0;  // Reset pending writes on new connection
  }
}

void SerialBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected reason=%d", reason);
  if(instance){
    instance->_isDeviceConnected = false;
    instance->_connectionHandle = 0xFFFF;  // Clear connection handle (BLE_CONN_HANDLE_INVALID)
    instance->_pending_writes = 0;  // Reset pending writes on disconnect
    // Don't manually restart advertising - let restartOnDisconnect(true) handle it
    // This prevents conflicts with iOS rapid reconnection attempts
  }
}

void SerialBLEInterface::onSecured(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: onSecured");
  if(instance){
    // Connection handle already set in onConnect(), no need to set again
    instance->_isDeviceConnected = true;
    // no need to stop advertising on connect, as the ble stack does this automatically
  }
}

bool SerialBLEInterface::onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing passkey request match=%d", match_request);
  (void)connection_handle;
  (void)passkey;
  // Accept pairing by returning true
  return true;
}

void SerialBLEInterface::onPairingComplete(uint16_t connection_handle, uint8_t auth_status) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing complete status=%d", auth_status);
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS) {
    BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing successful");
  } else {
    BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing failed, disconnecting");
    if (instance && instance->isConnectionHandleValid()) {
      Bluefruit.disconnect(connection_handle);
    }
  }
}

void SerialBLEInterface::onBLEEvent(ble_evt_t* evt) {
  // Hook into SoftDevice events to track TX completion
  if (!instance) return;

  switch(evt->header.evt_id) {
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      // Handle Value Notification transmission complete
      // Decrement pending writes counter by the number of completed transmissions
      if (instance->_pending_writes > 0) {
        uint8_t completed = evt->evt.gatts_evt.params.hvn_tx_complete.count;
        if (instance->_pending_writes >= completed) {
          instance->_pending_writes -= completed;
        } else {
          instance->_pending_writes = 0;
        }
        BLE_DEBUG_PRINTLN("TX complete: %d, pending now: %d", completed, instance->_pending_writes);
      }
      break;
    default:
      break;
  }
}

void SerialBLEInterface::begin(const char* device_name, uint32_t pin_code) {

  instance = this;

  char charpin[20];
  sprintf(charpin, "%d", pin_code);

  // Configure, begin, then clear advertising
  Bluefruit.autoConnLed(false);  // Disable connection LED
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();  // Begin before clearing advertising
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(device_name);

  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setPIN(charpin);
  Bluefruit.Security.setIOCaps(true, false, false);  // Display only (can show passkey, can't input)
  Bluefruit.Security.setPairPasskeyCallback(onPairingPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairingComplete);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Security.setSecuredCallback(onSecured);

  // Register event callback to track TX completion events
  Bluefruit.setEventCallback(onBLEEvent);

  // Configure and start the BLE Uart service
  bleuart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
  bleuart.begin();

  // Configure advertising payload once
  // Don't start yet - that happens in enable() or startAdv()
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  // Configure advertising parameters
  // Let SoftDevice handle auto-restart to prevent conflicts with iOS rapid reconnects
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);

}

void SerialBLEInterface::startAdv() {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: starting advertising");

  if(Bluefruit.Advertising.isRunning()){
    BLE_DEBUG_PRINTLN("SerialBLEInterface: already advertising");
    return;
  }

  Bluefruit.Advertising.start(0);  // 0 = Don't stop advertising after n seconds
}

void SerialBLEInterface::stopAdv() {

  BLE_DEBUG_PRINTLN("SerialBLEInterface: stopping advertising");
  
  // Only stop if running, otherwise an invalid state error is logged by BLE stack
  if(!Bluefruit.Advertising.isRunning()){
    return;
  }

  Bluefruit.Advertising.stop();

}

// ---------- public methods

void SerialBLEInterface::enable() {
  if (_isEnabled) return;

  _isEnabled = true;
  clearBuffers();

  // Re-enable auto-restart (in case disable() was called)
  Bluefruit.Advertising.restartOnDisconnect(true);
  startAdv();
}

void SerialBLEInterface::disconnect() {
  uint8_t connection_num = Bluefruit.connected();
  if (connection_num) {
    for (uint8_t i = 0; i < connection_num; i++) {
      Bluefruit.disconnect(i);
    }
    // Wait for disconnection to complete (prevents half-closed connections during rapid reconnects)
    while (Bluefruit.connected()) {
      yield();
    }
    BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnect completed");
  }
}

void SerialBLEInterface::disable() {
  _isEnabled = false;
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disable");

  disconnect();

  Bluefruit.Advertising.restartOnDisconnect(false);
  stopAdv();
  // Don't clear advertising data - it will be reconfigured on next begin()/enable()
}

size_t SerialBLEInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("writeFrame(), frame too big, len=%d", len);
    return 0;
  }

  if (isConnected() && len > 0) {
    if (send_queue_len >= FRAME_QUEUE_SIZE) {
      BLE_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
      return 0;
    }

    send_queue[send_queue_len].len = len;
    memcpy(send_queue[send_queue_len].buf, src, len);
    send_queue_len++;

    return len;
  }
  return 0;
}

bool SerialBLEInterface::isWriteBusy() const {
  // Check if we have pending writes in the SoftDevice queue
  return _pending_writes >= MAX_PENDING_WRITES;
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[]) {
  if (send_queue_len > 0 && _pending_writes < MAX_PENDING_WRITES) {
    if (isConnectionHandleValid() && bleuart.notifyEnabled(_connectionHandle)) {
      size_t written = bleuart.write(send_queue[0].buf, send_queue[0].len);
      if (written > 0) {
        _pending_writes++;
        BLE_DEBUG_PRINTLN("writeBytes: sz=%d, hdr=%d, pending=%d",
                         (uint32_t)send_queue[0].len, (uint32_t)send_queue[0].buf[0],
                         _pending_writes);

        send_queue_len--;
        for (int i = 0; i < send_queue_len; i++) {
          send_queue[i] = send_queue[i + 1];
        }
      } else {
        BLE_DEBUG_PRINTLN("writeBytes failed, pending=%d", _pending_writes);
      }
    }
  } else {
    int len = bleuart.available();
    if (len > 0 && len <= MAX_FRAME_SIZE) {
      bleuart.readBytes(dest, len);
      BLE_DEBUG_PRINTLN("readBytes: sz=%d, hdr=%d", len, (uint32_t) dest[0]);
      return len;
    }
  }
  return 0;
}

bool SerialBLEInterface::isConnected() const {
  if (!_isDeviceConnected) return false;
  if (isConnectionHandleValid()) {
    return Bluefruit.connected(_connectionHandle);
  }
  return false;
}
