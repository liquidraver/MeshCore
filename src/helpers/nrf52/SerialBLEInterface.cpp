#include "SerialBLEInterface.h"
#include <string.h> // For memcpy
#include "ble_gap.h"  // For SoftDevice GAP functions (sd_ble_gap_*)

static SerialBLEInterface* instance;

void SerialBLEInterface::onConnect(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: connected handle=0x%04X", connection_handle);
  if (instance) {
    // Defensive: If already connected with a different handle, disconnect the old one
    if (instance->isConnectionHandleValid() && instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: New connection with different handle! Old=0x%04X, New=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      Bluefruit.disconnect(instance->_connectionHandle);
    }
    instance->_connectionHandle = connection_handle;  // Store connection handle
    instance->_isDeviceConnected = false;
    instance->_pending_writes = 0;  // Reset pending writes on new connection
    instance->clearBuffers();  // Clear any stale queued frames
  }
}

void SerialBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected handle=0x%04X reason=%d", connection_handle, reason);
  if(instance){
    // Only process disconnect if it matches our stored handle (defensive check)
    if (instance->isConnectionHandleValid() && instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: Disconnect event for wrong handle! Stored=0x%04X, Event=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      return;
    }
    instance->_isDeviceConnected = false;
    instance->_connectionHandle = 0xFFFF;  // Clear connection handle (BLE_CONN_HANDLE_INVALID)
    instance->_pending_writes = 0;   // Reset pending writes on disconnect

    // Aggressively drop any queued frames so the next connection starts clean
    instance->send_queue_len = 0;

    // Don't manually restart advertising - let restartOnDisconnect(true) handle it
    // This prevents conflicts with iOS rapid reconnection attempts
  }
}

void SerialBLEInterface::onSecured(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: onSecured handle=0x%04X", connection_handle);
  if(instance){
    // Validate connection handle matches
    if (!instance->isConnectionHandleValid() || instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: onSecured with mismatched handle! Stored=0x%04X, Event=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      return;
    }
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
  // Hook into SoftDevice events to track TX completion and handle iOS 13+ requests
  if (!instance) return;

  // Extract connection handle based on event type
  uint16_t conn_handle = 0xFFFF;
  if (evt->header.evt_id >= BLE_GAP_EVT_BASE && evt->header.evt_id <= BLE_GAP_EVT_LAST) {
    conn_handle = evt->evt.gap_evt.conn_handle;
  } else if (evt->header.evt_id >= BLE_GATTS_EVT_BASE && evt->header.evt_id <= BLE_GATTS_EVT_LAST) {
    conn_handle = evt->evt.gatts_evt.conn_handle;
  }

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

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
      // iOS 13+ sends this - we MUST respond or SoftDevice will assert/crash
      BLE_DEBUG_PRINTLN("CONN_PARAM_UPDATE_REQUEST: handle=0x%04X, min_interval=%d, max_interval=%d, latency=%d, timeout=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.min_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.max_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.slave_latency,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.conn_sup_timeout);
      
      // Validate connection handle
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: CONN_PARAM_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      // Accept iOS's requested parameters by calling with NULL (uses PPCP from GAP service)
      // Alternatively, we could use the requested parameters directly
      uint32_t err_code = sd_ble_gap_conn_param_update(conn_handle, NULL);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted CONN_PARAM_UPDATE_REQUEST (using PPCP)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept CONN_PARAM_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
      // Connection parameters were updated
      BLE_DEBUG_PRINTLN("CONN_PARAM_UPDATE: handle=0x%04X, min_interval=%d, max_interval=%d, latency=%d, timeout=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout);
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
      // iOS 13+ may request PHY changes (1M/2M/Coded)
      BLE_DEBUG_PRINTLN("PHY_UPDATE_REQUEST: handle=0x%04X, tx_phys=0x%02X, rx_phys=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.tx_phys,
                       evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.rx_phys);
      
      // Validate connection handle
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: PHY_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      // Accept iOS's PHY preferences (use AUTO to let SoftDevice choose best)
      ble_gap_phys_t phy_params = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
      err_code = sd_ble_gap_phy_update(conn_handle, &phy_params);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted PHY_UPDATE_REQUEST (AUTO)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept PHY_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;

    case BLE_GAP_EVT_PHY_UPDATE:
      // PHY update completed
      BLE_DEBUG_PRINTLN("PHY_UPDATE: handle=0x%04X, status=0x%02X, tx_phy=0x%02X, rx_phy=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.phy_update.status,
                       evt->evt.gap_evt.params.phy_update.tx_phy,
                       evt->evt.gap_evt.params.phy_update.rx_phy);
      break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
      // iOS 13+ may request data length changes
      BLE_DEBUG_PRINTLN("DATA_LENGTH_UPDATE_REQUEST: handle=0x%04X, max_tx_octets=%d, max_tx_time=%d, max_rx_octets=%d, max_rx_time=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_tx_octets,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_tx_time,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_rx_octets,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_rx_time);
      
      // Validate connection handle
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: DATA_LENGTH_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      // Accept iOS's data length preferences (use AUTO to let SoftDevice choose best)
      err_code = sd_ble_gap_data_length_update(conn_handle, NULL, NULL);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted DATA_LENGTH_UPDATE_REQUEST (AUTO)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept DATA_LENGTH_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
      // Data length update completed
      BLE_DEBUG_PRINTLN("DATA_LENGTH_UPDATE: handle=0x%04X, max_tx_octets=%d, max_tx_time=%d, max_rx_octets=%d, max_rx_time=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time);
      break;

    case BLE_GAP_EVT_TIMEOUT:
      // Connection timeout
      BLE_DEBUG_PRINTLN("GAP_TIMEOUT: handle=0x%04X, src=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.timeout.src);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // ATT timeout (peer didn't respond)
      BLE_DEBUG_PRINTLN("GATTS_TIMEOUT: handle=0x%04X, src=0x%02X",
                       conn_handle,
                       evt->evt.gatts_evt.params.timeout.src);
      break;

    default:
      // Log unhandled events for debugging (only in debug mode to avoid spam)
      #ifdef BLE_DEBUG_LOGGING
      if (evt->header.evt_id >= BLE_GAP_EVT_BASE && evt->header.evt_id <= BLE_GAP_EVT_LAST) {
        BLE_DEBUG_PRINTLN("Unhandled GAP event: 0x%02X handle=0x%04X", evt->header.evt_id, conn_handle);
      }
      #endif
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
    if (_isDeviceConnected && isConnectionHandleValid() && bleuart.notifyEnabled(_connectionHandle)) {
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
    // Only process incoming data if connection is fully secured
    if (_isDeviceConnected) {
      int avail = bleuart.available();
      if (avail > 0) {
        // Cap read to avoid overflowing caller's buffer (MAX_FRAME_SIZE)
        int got = bleuart.readBytes(dest, avail > MAX_FRAME_SIZE ? MAX_FRAME_SIZE : avail);

        // If more data is pending, drain the excess to prevent buffer blocking
        if (avail > MAX_FRAME_SIZE) {
          uint8_t discard[32];
          int remaining = avail - got;
          while (remaining > 0) {
            int chunk = remaining < (int)sizeof(discard) ? remaining : (int)sizeof(discard);
            int drained = bleuart.readBytes(discard, chunk);
            if (drained <= 0) break;
            remaining -= drained;
          }
          BLE_DEBUG_PRINTLN("WARN: BLE RX overflow: avail=%d, read=%d, drained=%d", avail, got, avail - got - remaining);
        }

        BLE_DEBUG_PRINTLN("readBytes: sz=%d, hdr=%d", got, (uint32_t) dest[0]);
        return got;
      }
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
