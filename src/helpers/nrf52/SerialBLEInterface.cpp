#include "SerialBLEInterface.h"
#include <string.h>
#include "ble_gap.h"

static SerialBLEInterface* instance;

void SerialBLEInterface::onConnect(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: connected handle=0x%04X", connection_handle);
  if (instance) {
    if (instance->isConnectionHandleValid() && instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: New connection with different handle! Old=0x%04X, New=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      Bluefruit.disconnect(instance->_connectionHandle);
    }
    instance->_connectionHandle = connection_handle;
    instance->_isDeviceConnected = false;
    instance->_pending_writes = 0;
    instance->clearBuffers();
  }
}

void SerialBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected handle=0x%04X reason=%d", connection_handle, reason);
  if(instance){
    if (instance->isConnectionHandleValid() && instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: Disconnect event for wrong handle! Stored=0x%04X, Event=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      return;
    }
    instance->_isDeviceConnected = false;
    instance->_connectionHandle = 0xFFFF;
    instance->_pending_writes = 0;

    instance->send_queue_len = 0;
  }
}

void SerialBLEInterface::onSecured(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: onSecured handle=0x%04X", connection_handle);
  if(instance){
    if (!instance->isConnectionHandleValid() || instance->_connectionHandle != connection_handle) {
      BLE_DEBUG_PRINTLN("WARN: onSecured with mismatched handle! Stored=0x%04X, Event=0x%04X", 
                       instance->_connectionHandle, connection_handle);
      return;
    }
    instance->_isDeviceConnected = true;
  }
}

bool SerialBLEInterface::onPairingPasskey(uint16_t connection_handle, uint8_t const passkey[6], bool match_request) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing passkey request match=%d", match_request);
  (void)connection_handle;
  (void)passkey;
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
  if (!instance) return;
  uint16_t conn_handle = 0xFFFF;
  if (evt->header.evt_id >= BLE_GAP_EVT_BASE && evt->header.evt_id <= BLE_GAP_EVT_LAST) {
    conn_handle = evt->evt.gap_evt.conn_handle;
  } else if (evt->header.evt_id >= BLE_GATTS_EVT_BASE && evt->header.evt_id <= BLE_GATTS_EVT_LAST) {
    conn_handle = evt->evt.gatts_evt.conn_handle;
  }

  switch(evt->header.evt_id) {
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
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

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
      BLE_DEBUG_PRINTLN("CONN_PARAM_UPDATE_REQUEST: handle=0x%04X, min_interval=%d, max_interval=%d, latency=%d, timeout=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.min_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.max_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.slave_latency,
                       evt->evt.gap_evt.params.conn_param_update_request.conn_params.conn_sup_timeout);
      
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: CONN_PARAM_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      uint32_t err_code = sd_ble_gap_conn_param_update(conn_handle, NULL);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted CONN_PARAM_UPDATE_REQUEST (using PPCP)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept CONN_PARAM_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;
    }

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
      BLE_DEBUG_PRINTLN("CONN_PARAM_UPDATE: handle=0x%04X, min_interval=%d, max_interval=%d, latency=%d, timeout=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency,
                       evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout);
      break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
      BLE_DEBUG_PRINTLN("PHY_UPDATE_REQUEST: handle=0x%04X, tx_phys=0x%02X, rx_phys=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.tx_phys,
                       evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.rx_phys);
      
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: PHY_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      ble_gap_phys_t phy_params = { BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO };
      uint32_t err_code = sd_ble_gap_phy_update(conn_handle, &phy_params);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted PHY_UPDATE_REQUEST (AUTO)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept PHY_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;
    }

    case BLE_GAP_EVT_PHY_UPDATE:
      BLE_DEBUG_PRINTLN("PHY_UPDATE: handle=0x%04X, status=0x%02X, tx_phy=0x%02X, rx_phy=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.phy_update.status,
                       evt->evt.gap_evt.params.phy_update.tx_phy,
                       evt->evt.gap_evt.params.phy_update.rx_phy);
      break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: {
      BLE_DEBUG_PRINTLN("DATA_LENGTH_UPDATE_REQUEST: handle=0x%04X, max_tx_octets=%d, max_tx_time_us=%d, max_rx_octets=%d, max_rx_time_us=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_tx_octets,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_tx_time_us,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_rx_octets,
                       evt->evt.gap_evt.params.data_length_update_request.peer_params.max_rx_time_us);
      
      if (!instance->isConnectionHandleValid() || instance->_connectionHandle != conn_handle) {
        BLE_DEBUG_PRINTLN("WARN: DATA_LENGTH_UPDATE_REQUEST with mismatched handle! Stored=0x%04X, Event=0x%04X",
                         instance->_connectionHandle, conn_handle);
        return;
      }
      
      uint32_t err_code = sd_ble_gap_data_length_update(conn_handle, NULL, NULL);
      if (err_code == NRF_SUCCESS) {
        BLE_DEBUG_PRINTLN("Accepted DATA_LENGTH_UPDATE_REQUEST (AUTO)");
      } else {
        BLE_DEBUG_PRINTLN("ERROR: Failed to accept DATA_LENGTH_UPDATE_REQUEST: 0x%08X", err_code);
      }
      break;
    }

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
      BLE_DEBUG_PRINTLN("DATA_LENGTH_UPDATE: handle=0x%04X, max_tx_octets=%d, max_tx_time_us=%d, max_rx_octets=%d, max_rx_time_us=%d",
                       conn_handle,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets,
                       evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);
      break;

    case BLE_GAP_EVT_TIMEOUT:
      BLE_DEBUG_PRINTLN("GAP_TIMEOUT: handle=0x%04X, src=0x%02X",
                       conn_handle,
                       evt->evt.gap_evt.params.timeout.src);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      BLE_DEBUG_PRINTLN("GATTS_TIMEOUT: handle=0x%04X, src=0x%02X",
                       conn_handle,
                       evt->evt.gatts_evt.params.timeout.src);
      break;

    default:
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

  Bluefruit.autoConnLed(false);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(device_name);

  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setPIN(charpin);
  Bluefruit.Security.setIOCaps(true, false, false);
  Bluefruit.Security.setPairPasskeyCallback(onPairingPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairingComplete);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Security.setSecuredCallback(onSecured);

  Bluefruit.setEventCallback(onBLEEvent);

  bleuart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
  bleuart.begin();

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(bleuart);

  Bluefruit.ScanResponse.addName();

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

  Bluefruit.Advertising.start(0);
}

void SerialBLEInterface::stopAdv() {

  BLE_DEBUG_PRINTLN("SerialBLEInterface: stopping advertising");
  
  if(!Bluefruit.Advertising.isRunning()){
    return;
  }

  Bluefruit.Advertising.stop();

}

void SerialBLEInterface::enable() {
  if (_isEnabled) return;

  _isEnabled = true;
  clearBuffers();

  Bluefruit.Advertising.restartOnDisconnect(true);
  startAdv();
}

void SerialBLEInterface::disconnect() {
  uint8_t connection_num = Bluefruit.connected();
  if (connection_num) {
    for (uint8_t i = 0; i < connection_num; i++) {
      Bluefruit.disconnect(i);
    }
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
    if (_isDeviceConnected) {
      int avail = bleuart.available();
      if (avail > 0) {
        int got = bleuart.readBytes(dest, avail > MAX_FRAME_SIZE ? MAX_FRAME_SIZE : avail);

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
