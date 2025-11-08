#include "SerialBLEInterface.h"

static SerialBLEInterface* instance;

constexpr uint16_t kHighThroughputMinInterval = 6;   // 7.5 ms
constexpr uint16_t kHighThroughputMaxInterval = 12;  // 15 ms
constexpr uint16_t kLowPowerMinInterval       = 40;  // 50 ms
constexpr uint16_t kLowPowerMaxInterval       = 80;  // 100 ms
constexpr uint16_t kConnectionTimeout         = 400; // 4 s

void SerialBLEInterface::onConnect(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: connected");
  requestHighThroughput(connection_handle);
  // we now set _isDeviceConnected=true in onSecured callback instead
}

void SerialBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected reason=%d", reason);
  if(instance){
    instance->_isDeviceConnected = false;
    instance->startAdv();
  }
}

void SerialBLEInterface::onSecured(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: onSecured");
  if(instance){
    instance->_isDeviceConnected = true;
    requestLowPower(connection_handle);
    // no need to stop advertising on connect, as the ble stack does this automatically
  }
}

bool SerialBLEInterface::onPairPasskey(uint16_t conn_handle, uint8_t const passkey[6], bool match_request) {
  (void)conn_handle;
  (void)match_request;
  char displayed[7] = {0};
  memcpy(displayed, passkey, 6);
  BLE_DEBUG_PRINTLN("SerialBLEInterface: passkey %s", displayed);
  return true;
}

void SerialBLEInterface::onPairComplete(uint16_t conn_handle, uint8_t auth_status) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: pair status=%d", auth_status);
  if (auth_status == BLE_GAP_SEC_STATUS_SUCCESS) {
    requestLowPower(conn_handle);
  }
}

void SerialBLEInterface::begin(const char* device_name, uint32_t pin_code) {

  instance = this;

  char charpin[20];
  sprintf(charpin, "%d", pin_code);

  Bluefruit.autoConnLed(false);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName(device_name);
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  dis.setManufacturer("MeshCore");
  dis.setModel(device_name);
  dis.begin();

  bas.begin();
  bas.write(100);

  Bluefruit.Security.setMITM(true);
  Bluefruit.Security.setPIN(charpin);
  Bluefruit.Security.setPairPasskeyCallback(onPairPasskey);
  Bluefruit.Security.setPairCompleteCallback(onPairComplete);
  bleuart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Security.setSecuredCallback(onSecured);

  // To be consistent OTA DFU should be added first if it exists
  //bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();
  
}

void SerialBLEInterface::startAdv() {

  BLE_DEBUG_PRINTLN("SerialBLEInterface: starting advertising");
  
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addTxPower();
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

}

void SerialBLEInterface::stopAdv() {

  BLE_DEBUG_PRINTLN("SerialBLEInterface: stopping advertising");
  
  // we only want to stop advertising if it's running, otherwise an invalid state error is logged by ble stack
  if(!Bluefruit.Advertising.isRunning()){
    return;
  }

  // stop advertising
  Bluefruit.Advertising.stop();

}

// ---------- public methods

void SerialBLEInterface::enable() { 
  if (_isEnabled) return;

  _isEnabled = true;
  clearBuffers();

  // Start advertising
  startAdv();
}

void SerialBLEInterface::disable() {
  _isEnabled = false;
  BLE_DEBUG_PRINTLN("SerialBLEInterface::disable");

#ifdef RAK_BOARD
  Bluefruit.disconnect(Bluefruit.connHandle());
#else
  uint16_t conn_id;
  if (Bluefruit.getConnectedHandles(&conn_id, 1) > 0) {
    Bluefruit.disconnect(conn_id);
  }
#endif

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.stop();

  stopAdv();
}

void SerialBLEInterface::requestHighThroughput(uint16_t conn_handle) {
  if (conn_handle == BLE_CONN_HANDLE_INVALID) {
    return;
  }
  ble_gap_conn_params_t params = {
    .min_conn_interval = kHighThroughputMinInterval,
    .max_conn_interval = kHighThroughputMaxInterval,
    .slave_latency     = 0,
    .conn_sup_timeout  = kConnectionTimeout
  };
  sd_ble_gap_conn_param_update(conn_handle, &params);
}

void SerialBLEInterface::requestLowPower(uint16_t conn_handle) {
  if (conn_handle == BLE_CONN_HANDLE_INVALID) {
    return;
  }
  ble_gap_conn_params_t params = {
    .min_conn_interval = kLowPowerMinInterval,
    .max_conn_interval = kLowPowerMaxInterval,
    .slave_latency     = 0,
    .conn_sup_timeout  = kConnectionTimeout
  };
  sd_ble_gap_conn_param_update(conn_handle, &params);
}

size_t SerialBLEInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    BLE_DEBUG_PRINTLN("writeFrame(), frame too big, len=%d", len);
    return 0;
  }

  if (_isDeviceConnected && len > 0) {
    if (send_queue_len >= FRAME_QUEUE_SIZE) {
      BLE_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
      return 0;
    }

    send_queue[send_queue_len].len = len;  // add to send queue
    memcpy(send_queue[send_queue_len].buf, src, len);
    send_queue_len++;

    return len;
  }
  return 0;
}

#define  BLE_WRITE_MIN_INTERVAL   60

bool SerialBLEInterface::isWriteBusy() const {
  return millis() < _last_write + BLE_WRITE_MIN_INTERVAL;   // still too soon to start another write?
}

size_t SerialBLEInterface::checkRecvFrame(uint8_t dest[]) {
  if (send_queue_len > 0   // first, check send queue
    && millis() >= _last_write + BLE_WRITE_MIN_INTERVAL    // space the writes apart
  ) {
    _last_write = millis();
    bleuart.write(send_queue[0].buf, send_queue[0].len);
    BLE_DEBUG_PRINTLN("writeBytes: sz=%d, hdr=%d", (uint32_t)send_queue[0].len, (uint32_t) send_queue[0].buf[0]);

    send_queue_len--;
    for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
      send_queue[i] = send_queue[i + 1];
    }
  } else {
    int len = bleuart.available();
    if (len > 0) {
      bleuart.readBytes(dest, len);
      BLE_DEBUG_PRINTLN("readBytes: sz=%d, hdr=%d", len, (uint32_t) dest[0]);
      return len;
    }
  }
  return 0;
}

bool SerialBLEInterface::isConnected() const {
  return _isDeviceConnected;
}
