#include "SerialBLEInterface.h"
#include <algorithm>

static SerialBLEInterface* instance;

void SerialBLEInterface::onDisconnect(uint16_t connection_handle, uint8_t reason) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: disconnected reason=%d", reason);
  if(instance){
    instance->_isDeviceConnected = false;
    instance->clearBuffers();
    if (instance->_isEnabled) {
      instance->startAdv();
    } else {
      Bluefruit.Advertising.stop();
    }
  }
}

void SerialBLEInterface::onSecured(uint16_t connection_handle) {
  BLE_DEBUG_PRINTLN("SerialBLEInterface: onSecured");
  if(instance){
    instance->_isDeviceConnected = true;
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
  bleuart.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);

  Bluefruit.Periph.setDisconnectCallback(onDisconnect);
  Bluefruit.Security.setSecuredCallback(onSecured);

  // To be consistent OTA DFU should be added first if it exists
  //bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();
  bleuart.setRxCallback(onBleUartRX);
  
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
  Bluefruit.Security.setPairPasskeyCallback(onPairPasskey);

  // Start advertising
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  startAdv();
}

void SerialBLEInterface::disable() {
  _isEnabled = false;
  BLE_DEBUG_PRINTLN("SerialBLEInterface::disable");
  clearBuffers();
  _isDeviceConnected = false;

#ifdef RAK_BOARD
  Bluefruit.disconnect(Bluefruit.connHandle());
#else
  uint16_t conn_id;
  if (Bluefruit.getConnectedHandles(&conn_id, 1) > 0) {
    Bluefruit.disconnect(conn_id);
  }
#endif

  Bluefruit.Security.setPairPasskeyCallback(onRejectPair);

  Bluefruit.Advertising.restartOnDisconnect(false);
  stopAdv();
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
    BLE_DEBUG_PRINTLN("SerialBLEInterface writeBytes: sz=%d hdr=%d", (uint32_t)send_queue[0].len, (uint32_t) send_queue[0].buf[0]);

    send_queue_len--;
    for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
      send_queue[i] = send_queue[i + 1];
    }
  } else {
    if (recv_queue_len > 0) {
      uint8_t len = recv_queue[0].len;
      memcpy(dest, recv_queue[0].buf, len);
      recv_queue_len--;
      for (int i = 0; i < recv_queue_len; i++) {
        recv_queue[i] = recv_queue[i + 1];
      }
      BLE_DEBUG_PRINTLN("SerialBLEInterface readBytes: sz=%d hdr=%d", len, (uint32_t) dest[0]);
      return len;
    }
  }

  return 0;
}

bool SerialBLEInterface::isConnected() const {
  return _isDeviceConnected;
}

bool SerialBLEInterface::onRejectPair(uint16_t conn_handle, uint8_t const passkey[6], bool match_request) {
  (void)passkey;
  (void)match_request;
  BLE_DEBUG_PRINTLN("SerialBLEInterface: pairing rejected while disabled");
  Bluefruit.disconnect(conn_handle);
  return false;
}

void SerialBLEInterface::onBleUartRX(uint16_t conn_handle) {
  (void)conn_handle;
  if (!instance) {
    return;
  }
  BLE_DEBUG_PRINTLN("SerialBLEInterface: RX activity");
  while (instance->bleuart.available()) {
    if (instance->recv_queue_len >= FRAME_QUEUE_SIZE) {
    BLE_DEBUG_PRINTLN("SerialBLEInterface: recv queue full (len=%d avail=%d)", instance->recv_queue_len, instance->bleuart.available());
      while (instance->bleuart.available()) {
        instance->bleuart.read();
      }
      break;
    }
    size_t available = instance->bleuart.available();
    uint8_t chunk = static_cast<uint8_t>(std::min<size_t>(available, MAX_FRAME_SIZE));
    SerialBLEInterface::Frame& frame = instance->recv_queue[instance->recv_queue_len];
    frame.len = chunk;
    instance->bleuart.readBytes(frame.buf, chunk);
    instance->recv_queue_len++;
    BLE_DEBUG_PRINTLN("SerialBLEInterface: queued RX chunk=%d queue=%d", chunk, instance->recv_queue_len);
  }
}
