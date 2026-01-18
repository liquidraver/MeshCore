#include "SerialWifiInterface.h"
#include <WiFi.h>

void SerialWifiInterface::begin(int port) {
  WIFI_DEBUG_PRINTLN("begin() port=%d", port);
  server.begin(port);
  WIFI_DEBUG_PRINTLN("server.begin() done");
}

void SerialWifiInterface::enable() {
  if (_isEnabled) return;
  _isEnabled = true;
  clearBuffers();
}

void SerialWifiInterface::disable() {
  _isEnabled = false;
}

size_t SerialWifiInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    WIFI_DEBUG_PRINTLN("writeFrame: too big len=%d", (int)len);
    return 0;
  }

  if (!_deviceConnected || len == 0) {
    return 0;
  }

  // Direct write - TCP handles buffering
  uint8_t hdr[3] = {'>', (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  client.write(hdr, 3);
  client.write(src, len);
  return len;
}

bool SerialWifiInterface::isWriteBusy() const {
  return false;  // TCP handles buffering, never busy
}

size_t SerialWifiInterface::checkRecvFrame(uint8_t dest[]) {
  // Check for new client connection
  WiFiClient newClient = server.available();
  if (newClient) {
    WIFI_DEBUG_PRINTLN("New client connecting");
    // Disconnect existing client
    _deviceConnected = false;
    client.stop();
    // Accept new client
    client = newClient;
  }

  // Track connection state
  if (client.connected()) {
    if (!_deviceConnected) {
      WIFI_DEBUG_PRINTLN("Connected");
      _deviceConnected = true;
      clearBuffers();
    }
  } else {
    if (_deviceConnected) {
      WIFI_DEBUG_PRINTLN("Disconnected");
      _deviceConnected = false;
      clearBuffers();
    }
  }

  if (!_deviceConnected) {
    return 0;
  }

  // Check for incoming data and accumulate in staging buffer
  int avail = client.available();
  if (avail > 0) {
    // Read as much as we can fit in staging buffer
    int space = sizeof(_rx_buf) - _rx_buf_len;
    if (space > 0) {
      int to_read = (avail < space) ? avail : space;
      int got = client.readBytes(_rx_buf + _rx_buf_len, to_read);
      _rx_buf_len += got;
    }
  }

  // Try to extract a complete frame from staging buffer
  // Frame format: marker (1) + length LSB (1) + length MSB (1) + payload
  if (_rx_buf_len >= 3) {
    uint16_t payload_len = _rx_buf[1] | (_rx_buf[2] << 8);

    // Sanity check
    if (payload_len > MAX_FRAME_SIZE) {
      WIFI_DEBUG_PRINTLN("RX: invalid frame len=%u, clearing buffer", payload_len);
      _rx_buf_len = 0;
      return 0;
    }

    uint16_t frame_len = 3 + payload_len;
    if (_rx_buf_len >= frame_len) {
      // Complete frame available - copy payload to dest
      memcpy(dest, _rx_buf + 3, payload_len);

      // Shift remaining data in staging buffer
      uint16_t remaining = _rx_buf_len - frame_len;
      if (remaining > 0) {
        memmove(_rx_buf, _rx_buf + frame_len, remaining);
      }
      _rx_buf_len = remaining;

      return payload_len;
    }
  }

  return 0;
}

bool SerialWifiInterface::isConnected() const {
  return _deviceConnected;
}
