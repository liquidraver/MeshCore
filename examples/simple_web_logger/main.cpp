#include <Arduino.h> // needed for PlatformIO
#include <Mesh.h>

// export PATH="/root/.platformio/penv/bin:$PATH"

#ifndef ESP32
  #error "Platform not supported."
#endif

#include <SPIFFS.h>
#include <WiFi.h>
#include "time.h"
#include <queue>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/IdentityStore.h>
#include <helpers/pingpong.h>
#include <helpers/timesync_monitor.h>
#include <RTClib.h>
#include <target.h>

#include "LoggerMeshTables.h"

/* ---------------------------------- CONFIGURATION ------------------------------------- */

#define FIRMWARE_VER_TEXT   "v1.8.1"

// Compile-time check for PINGPONG_ENABLED
#ifdef PINGPONG_ENABLED
  // PINGPONG_ENABLED is defined
#else
  // PINGPONG_ENABLED is NOT defined
#endif

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     250
#endif
#ifndef LORA_SF
  #define LORA_SF     10
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER  20
#endif

#ifndef MAX_CONTACTS
  #define MAX_CONTACTS         300
#endif

#include <helpers/BaseChatMesh.h>

#define SEND_TIMEOUT_BASE_MILLIS          500
#define FLOOD_SEND_TIMEOUT_FACTOR         16.0f
#define DIRECT_SEND_PERHOP_FACTOR         6.0f
#define DIRECT_SEND_PERHOP_EXTRA_MILLIS   250

#define  PUBLIC_GROUP_PSK "izOH6cXN6mrJ5e26oRXNcg=="

static bool ntpSynced = false;
const char* ntpServer = "91.82.109.180";
const long  gmtOffset_sec = 60 * 60 * 3;
const int   daylightOffset_sec = 3600;
const unsigned long ntpFirstSyncDelay = 10 * 1000;        // 10 seconds after boot
const unsigned long ntpSecondSyncInterval = 5 * 60 * 1000; // 5 minutes after first
const unsigned long ntpRegularSyncInterval = 2 * 60 * 60 * 1000; // 2 hours thereafter
unsigned long ntpNext = 0;
static int ntpSyncCount = 0;


// RTOS, wifi thread
TaskHandle_t WiFiTask;
void WiFiTaskCode(void* pvParameters);

void task_sleep(uint32_t ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct {
  std::queue<char*> queue;
  unsigned discarded;

  void push(String str) {
    unsigned bef = ESP.getFreeHeap();

    char* msgData = new char[str.length() + 1];
    str.toCharArray(msgData, str.length() + 1);
  
    // discard old
    while (queue.size() > 32) {
      discarded++;
      char* ptr = queue.front();
      Serial.printf("Discarded message (%u) %s\n", discarded, ptr);
      queue.pop();
      delete[] ptr;
    }
    queue.push(msgData);

    unsigned aft = ESP.getFreeHeap();
  }

  void push(const JsonDocument& doc) {
    String postData;
    serializeJson(doc, postData);
    push(postData);
  }

  size_t size() { return queue.size(); }
  char* front() { return queue.front(); }
  void  pop()   { queue.pop(); }
} messageQueue;

unsigned long getTimestamp() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return(0);
  }
  time(&now);
  return now;
}

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

/* -------------------------------------------------------------------------------------- */

struct NodePrefs {  // persisted to file
  float airtime_factor;
  char node_name[32];
  double node_lat, node_lon;
  float freq;
  uint8_t tx_power_dbm;
  float bw;
  uint8_t sf;
  uint8_t cr;
  uint8_t unused[2];
};

struct WiFiPrefs {
  char ssid[33];
  char password[64];
};

struct LogPrefs {
  uint16_t version;
  uint32_t selfreport;
  char auth[32];
  char url[256];
  uint8_t doraw;
  uint8_t dofwd;
};

// Retry system for channel messages
struct PendingRetry {
  mesh::Packet* packet;
  unsigned long retry_time;
  uint8_t retry_count;
  bool is_active;
};

static const int MAX_PENDING_RETRIES = 8;
PendingRetry pending_retries[MAX_PENDING_RETRIES];


#define MAX_ADMINS 8
struct AdminPrefs {
  uint16_t version;
  uint16_t count;
  uint8_t admin_keys[MAX_ADMINS][PUB_KEY_SIZE];
};

class MyMesh : public BaseChatMesh, ContactVisitor {
  FILESYSTEM* _fs;
  NodePrefs _prefs;
  WiFiPrefs _wifi;
  LogPrefs _logp;
  AdminPrefs _admin;
  LoggerMeshTables* _tables;
  uint32_t expected_ack_crc;
  ChannelDetails* _public;
  ChannelDetails* _ping_channel;
  unsigned long last_msg_sent;
  ContactInfo* curr_recipient;
  char command[512+10];
  uint8_t tmp_buf[256];
  char hex_buf[512];

  // debug toggle flag
  bool m_debugPrint = false;
  
  // Retry system helper functions for channel messages
  void scheduleRetry(mesh::Packet* packet, uint32_t delay_ms) {
    for (int i = 0; i < MAX_PENDING_RETRIES; i++) {
      if (!pending_retries[i].is_active) {
        pending_retries[i].packet = packet;
        pending_retries[i].retry_time = millis() + delay_ms;
        pending_retries[i].retry_count = 1;
        pending_retries[i].is_active = true;
        
        MESH_DEBUG_PRINTLN("[RETRY] Scheduled retry #%d in %dms", 1, delay_ms);
        return;
      }
    }
    MESH_DEBUG_PRINTLN("[RETRY] ERROR: No free retry slots available");
    releasePacket(packet);  // Release packet if we can't schedule retry
  }
  
  void processRetries() {
    for (int i = 0; i < MAX_PENDING_RETRIES; i++) {
      if (pending_retries[i].is_active && millis() >= pending_retries[i].retry_time) {
        mesh::Packet* packet = pending_retries[i].packet;
        uint8_t retry_count = pending_retries[i].retry_count;
        
        // Check if we heard repetition of this packet
        if (_tables->wasPacketHeard(packet)) {
          MESH_DEBUG_PRINTLN("[RETRY] Packet was heard, canceling retry #%d", retry_count);
          pending_retries[i].is_active = false;
          releasePacket(packet);
        } else {
          // Retry the packet
          MESH_DEBUG_PRINTLN("[RETRY] Retrying packet #%d (not heard)", retry_count);
          sendFlood(packet, 0);  // Send immediately
          // Note: Don't track retries - they're already tracked from original send
          
          if (retry_count < 2) {  // Allow one more retry
            pending_retries[i].retry_count++;
            pending_retries[i].retry_time = millis() + getRNG()->nextInt(2000, 4001);  // 2-4 seconds
          } else {
            MESH_DEBUG_PRINTLN("[RETRY] Max retries reached, giving up");
            pending_retries[i].is_active = false;
            releasePacket(packet);
          }
        }
      }
    }
  }
  
  // Override BaseChatMesh::sendChannelMessage to use retry system
  bool sendChannelMessage(const mesh::GroupChannel& channel, const char* message, const char* sender_name, uint32_t delay_ms = 0) override {
    uint8_t temp[5+MAX_TEXT_LEN+32];
    uint32_t timestamp = getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;  // TXT_TYPE_PLAIN

    // Format: <sender_name>: <message>
    sprintf((char *) &temp[5], "%s: %s", sender_name, message);
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

    int len = strlen((char *) &temp[5]);
    auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + len);
    if (!pkt) {
      MESH_DEBUG_PRINTLN("[CHANNEL-RETRY] ERROR: Failed to create group datagram (packet pool full?)");
      return false;
    }
    
    MESH_DEBUG_PRINTLN("[CHANNEL-RETRY] Sending channel message with %dms delay", delay_ms);
    
    // Send with retry system
    sendFlood(pkt, delay_ms);
    
    // Track packet AFTER sending (not before) to avoid false repetition detection
    _tables->trackSentPacket(pkt);
    scheduleRetry(pkt, delay_ms + getRNG()->nextInt(2000, 4001));  // Schedule retry check
    return true;
  }

  const char* getTypeName(uint8_t type) const {
    if (type == ADV_TYPE_CHAT) return "Chat";
    if (type == ADV_TYPE_REPEATER) return "Repeater";
    if (type == ADV_TYPE_ROOM) return "Room";
    return "??";  // unknown
  }

  void loadContacts() {
    if (_fs->exists("/contacts")) {
      File file = _fs->open("/contacts");
      if (file) {
        bool full = false;
        while (!full) {
          ContactInfo c;
          uint8_t pub_key[32];
          uint8_t unused;
          uint32_t reserved;

          bool success = (file.read(pub_key, 32) == 32);
          success = success && (file.read((uint8_t *) &c.name, 32) == 32);
          success = success && (file.read(&c.type, 1) == 1);
          success = success && (file.read(&c.flags, 1) == 1);
          success = success && (file.read(&unused, 1) == 1);
          success = success && (file.read((uint8_t *) &reserved, 4) == 4);
          success = success && (file.read((uint8_t *) &c.out_path_len, 1) == 1);
          success = success && (file.read((uint8_t *) &c.last_advert_timestamp, 4) == 4);
          success = success && (file.read(c.out_path, 64) == 64);
          c.gps_lat = c.gps_lon = 0;   // not yet supported

          if (!success) break;  // EOF

          c.id = mesh::Identity(pub_key);
          c.lastmod = 0;
          if (!addContact(c)) full = true;
        }
        file.close();
      }
    }
  }

  void deleteChannels() {
    if (_fs->exists("/channels2")) {
      _fs->remove("/channels2");
    }
  }

  // Helper function to convert hex PSK to base64
  bool hexPskToBase64(const char* hex_psk, char* base64_out, size_t base64_size) {
    uint8_t psk_bytes[32]; // Max 32 bytes (256-bit key)
    size_t hex_len = strlen(hex_psk);
    int psk_len = 0;
    
    if (hex_len == 32) {
      // 32 hex chars = 16 bytes (128-bit key)
      if (!mesh::Utils::fromHex(psk_bytes, 16, hex_psk)) return false;
      psk_len = 16;
    } else if (hex_len == 64) {
      // 64 hex chars = 32 bytes (256-bit key)
      if (!mesh::Utils::fromHex(psk_bytes, 32, hex_psk)) return false;
      psk_len = 32;
    } else {
      return false;
    }
    
    // Encode to base64 (process in groups of 3 bytes)
    const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int j = 0;
    int i = 0;
    
    while (i < psk_len) {
      uint32_t triple = psk_bytes[i++] << 16;
      int bytes_in_group = 1;
      
      if (i < psk_len) {
        triple |= psk_bytes[i++] << 8;
        bytes_in_group++;
      }
      if (i < psk_len) {
        triple |= psk_bytes[i++];
        bytes_in_group++;
      }
      
      base64_out[j++] = base64_chars[(triple >> 18) & 63];
      base64_out[j++] = base64_chars[(triple >> 12) & 63];
      
      if (bytes_in_group >= 2) {
        base64_out[j++] = base64_chars[(triple >> 6) & 63];
      } else {
        base64_out[j++] = '=';
      }
      
      if (bytes_in_group >= 3) {
        base64_out[j++] = base64_chars[triple & 63];
      } else {
        base64_out[j++] = '=';
      }
    }
    base64_out[j] = '\0';
    return true;
  }

  // Remove channel by index (removes from array and shifts remaining channels)
  bool removeChannelByIdx(int idx) {
#ifdef MAX_GROUP_CHANNELS
    if (idx < 0 || idx >= MAX_GROUP_CHANNELS) return false;
    
    ChannelDetails ch;
    if (!getChannel(idx, ch)) return false;  // Channel doesn't exist at this index
    
    // Find how many channels exist
    int max_idx = idx;
    ChannelDetails temp;
    while (getChannel(max_idx + 1, temp)) {
      max_idx++;
    }
    
    // Shift all channels after idx one position forward using getChannel/setChannel
    for (int i = idx; i < max_idx; i++) {
      if (getChannel(i + 1, temp)) {
        setChannel(i, temp);
      }
    }
    
    // Clear the last slot by setting an empty channel
    memset(&temp, 0, sizeof(ChannelDetails));
    setChannel(max_idx, temp);
    
    return true;
#else
    return false;
#endif
  }

  void loadChannels() {
    if (_fs->exists("/channels2")) {
      File file = _fs->open("/channels2");
      if (file) {
        bool full = false;
        uint8_t channel_idx = 0;
        while (!full) {
          ChannelDetails ch;
          uint8_t unused[4];

          bool success = (file.read(unused, 4) == 4);
          success = success && (file.read((uint8_t *)ch.name, 32) == 32);
          success = success && (file.read((uint8_t *)ch.channel.secret, 32) == 32);

          if (!success) break; // EOF

          if (setChannel(channel_idx, ch)) {
            channel_idx++;
          } else {
            full = true;
          }
        }
        file.close();
      }
    }
  }

  void saveChannels() {
    File file = _fs->open("/channels2", "w", true);
    if (file) {
      uint8_t channel_idx = 0;
      ChannelDetails ch;
      uint8_t unused[4];
      memset(unused, 0, 4);

      while (getChannel(channel_idx, ch)) {
        bool success = (file.write(unused, 4) == 4);
        success = success && (file.write((uint8_t *)ch.name, 32) == 32);
        success = success && (file.write((uint8_t *)ch.channel.secret, 32) == 32);

        if (!success) break; // write failed
        channel_idx++;
      }
      file.close();
    }
  }

  void loadAdmins() {
    if (_fs->exists("/admin_prefs")) {
      File file = _fs->open("/admin_prefs");
      if (file) {
        file.read((uint8_t *)&_admin, sizeof(_admin));
        file.close();
        
        if (_admin.version == 0) {
          _admin.version = 1;
          _admin.count = 0;
        }
        
        // Validate count
        if (_admin.count > MAX_ADMINS) {
          _admin.count = 0;
        }
      }
    } else {
      // Initialize empty admin list
      _admin.version = 1;
      _admin.count = 0;
      memset(_admin.admin_keys, 0, sizeof(_admin.admin_keys));
    }
  }

  void saveAdmins() {
    File file = _fs->open("/admin_prefs", "w", true);
    if (file) {
      file.write((const uint8_t *)&_admin, sizeof(_admin));
      file.close();
    }
  }

  bool isAdmin(const uint8_t* pub_key) {
    for (uint16_t i = 0; i < _admin.count; i++) {
      if (memcmp(pub_key, _admin.admin_keys[i], PUB_KEY_SIZE) == 0) {
        return true;
      }
    }
    return false;
  }

  bool addAdmin(const uint8_t* pub_key) {
    // Check if already admin
    if (isAdmin(pub_key)) {
      return false; // Already exists
    }
    
    // Check if we have space
    if (_admin.count >= MAX_ADMINS) {
      return false; // Full
    }
    
    // Add new admin
    memcpy(_admin.admin_keys[_admin.count], pub_key, PUB_KEY_SIZE);
    _admin.count++;
    saveAdmins();
    return true;
  }

  void saveContacts() {
    File file = _fs->open("/contacts", "w", true);
    if (file) {
      ContactsIterator iter;
      ContactInfo c;
      uint8_t unused = 0;
      uint32_t reserved = 0;

      while (iter.hasNext(this, c)) {
        bool success = (file.write(c.id.pub_key, 32) == 32);
        success = success && (file.write((uint8_t *) &c.name, 32) == 32);
        success = success && (file.write(&c.type, 1) == 1);
        success = success && (file.write(&c.flags, 1) == 1);
        success = success && (file.write(&unused, 1) == 1);
        success = success && (file.write((uint8_t *) &reserved, 4) == 4);
        success = success && (file.write((uint8_t *) &c.out_path_len, 1) == 1);
        success = success && (file.write((uint8_t *) &c.last_advert_timestamp, 4) == 4);
        success = success && (file.write(c.out_path, 64) == 64);

        if (!success) break;  // write failed
      }
      file.close();
    }
  }

public:
  void setClock(uint32_t timestamp, bool ntp) {
    uint32_t curr = getRTCClock()->getCurrentTime();
    if (timestamp > curr || ntp) {
      getRTCClock()->setCurrentTime(timestamp);
      if (!ntp && !ntpSynced) {
        Serial.println("   Synced local");
        timeval epoch = {timestamp, 0};
        settimeofday((const timeval*)&epoch, 0);
        // update local
      } else {
        ntpSynced = true;
      }
      Serial.println("   (OK - clock set!)");
    } else {
      Serial.println("   (ERR: clock cannot go backwards)");
    }
  }

  const NodePrefs* getNodePrefs() { return &_prefs; }
  const LogPrefs* getLogPrefs() { return &_logp; }
  const bool debugPrint() { return m_debugPrint; }

private:
  void importCard(const char* command) {
    while (*command == ' ') command++;   // skip leading spaces
    if (memcmp(command, "meshcore://", 11) == 0) {
      command += 11;  // skip the prefix
      char *ep = strchr(command, 0);  // find end of string
      while (ep > command) {
        ep--;
        if (mesh::Utils::isHexChar(*ep)) break;  // found tail end of card
        *ep = 0;  // remove trailing spaces and other junk
      }
      int len = strlen(command);
      if (len % 2 == 0) {
        len >>= 1;  // halve, for num bytes
        if (mesh::Utils::fromHex(tmp_buf, len, command)) {
          importContact(tmp_buf, len);
          return;
        }
      }
    }
    Serial.println("   error: invalid format");
  }

protected:
  float getAirtimeBudgetFactor() const override {
    return _prefs.airtime_factor;
  }

  int calcRxDelay(float score, uint32_t air_time) const override {
    return 0;  // disable rxdelay
  }

  bool allowPacketForward(const mesh::Packet* packet) override {
    return _logp.dofwd && _tables->hasSeen2(packet);
  }

  //void BaseChatMesh::onAdvertRecv(mesh::Packet* packet, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len)

  const char* type2str(int type) {
    if(type == PAYLOAD_TYPE_REQ) return "REQ";
    else if(type == PAYLOAD_TYPE_RESPONSE) return "RESPONSE";
    else if(type == PAYLOAD_TYPE_TXT_MSG) return "TXT_MSG";
    else if(type == PAYLOAD_TYPE_ACK) return "ACK";
    else if(type == PAYLOAD_TYPE_ADVERT) return "ADVERT";
    else if(type == PAYLOAD_TYPE_GRP_TXT) return "GRP_TXT";
    else if(type == PAYLOAD_TYPE_GRP_DATA) return "GRP_DATA";
    else if(type == PAYLOAD_TYPE_ANON_REQ) return "ANON_REQ";
    else if(type == PAYLOAD_TYPE_PATH) return "PATH";
    else if(type == PAYLOAD_TYPE_TRACE) return "TRACE";
    return "Unknown";
  } 

  mesh::DispatcherAction onRecvPacket(mesh::Packet* pkt) override {
    // send raw
    if (_logp.doraw) {
      int phType = (pkt->header >> PH_TYPE_SHIFT) & PH_TYPE_MASK;

      if (debugPrint()) {
        Serial.println("[RAW] Received packet:");
        Serial.printf("      header:          %u\n", pkt->header);
        Serial.printf("        route-type:    %u\n", pkt->header & PH_ROUTE_MASK); // 2 bits
        Serial.printf("        payload-type:  %s (%u)\n", type2str(phType), phType); // 4 bits
        Serial.printf("        payload-vers:  %u\n", (pkt->header >> PH_VER_SHIFT) & PH_VER_MASK); // 2 bits
        Serial.printf("      payload_len:     %u\n", pkt->payload_len);
        Serial.printf("      path_len:        %u\n", pkt->path_len);
        Serial.printf("      transport_codes: %u %u\n", pkt->transport_codes[0], pkt->transport_codes[1]);
        Serial.printf("      snr:             %i\n", pkt->_snr);
        Serial.println();
      }

      uint8_t hash[MAX_HASH_SIZE];
      pkt->calculatePacketHash(hash);

      char sender[(PUB_KEY_SIZE * 2) + 1];
      char path[(pkt->path_len * 2) + 1];
      char payload[(pkt->payload_len * 2) + 1];
      char strhash[MAX_HASH_SIZE * 2 + 1];

      mesh::Utils::toHex(sender, self_id.pub_key, PUB_KEY_SIZE);
      mesh::Utils::toHex(path, pkt->path, pkt->path_len);
      mesh::Utils::toHex(payload, pkt->payload, pkt->payload_len);
      mesh::Utils::toHex(strhash, hash, MAX_HASH_SIZE);

      JsonDocument doc;
      doc["version"] = 1;
      doc["type"] = "RAW";
      doc["reporter"] = sender;
      doc["recvtime"] = getRTCClock()->getCurrentTime();
      doc["hash"] = strhash;
      doc["packet"]["header"]["raw"] = pkt->header;
      doc["packet"]["header"]["route-type"] = pkt->header & PH_ROUTE_MASK;
      doc["packet"]["header"]["payload-type"] = (pkt->header >> PH_TYPE_SHIFT) & PH_TYPE_MASK;
      doc["packet"]["header"]["payload-version"] = (pkt->header >> PH_VER_SHIFT) & PH_VER_MASK;
      doc["packet"]["path"] = path;
      doc["packet"]["payload"] = payload;
      doc["packet"]["snr"] = pkt->_snr;
      messageQueue.push(doc);
    }

    return Mesh::onRecvPacket(pkt);
  }

  void onAdvertRecv(mesh::Packet* pkt, const mesh::Identity& id, uint32_t timestamp, const uint8_t* app_data, size_t app_data_len) {
    ContactInfo* from = lookupContactByPubKey(id.pub_key, PUB_KEY_SIZE);
    bool is_new = from == NULL;
    BaseChatMesh::onAdvertRecv(pkt, id, timestamp, app_data, app_data_len);  // chain to super impl
    from = lookupContactByPubKey(id.pub_key, PUB_KEY_SIZE);

    if (!from) {
      Serial.println("ERROR: onAdvertRecv: Contact not found!");
    }

    AdvertDataParser parser(app_data, app_data_len);
    if (!(parser.isValid() && parser.hasName())) {
      Serial.printf("ERROR: onAdvertRecv: invalid app_data, or name is missing: len=%d\n", app_data_len);
      return;
    }

#ifdef TIMESYNC_MONITOR_ENABLED
    // Monitor time sync status (only if our node has good time)
    uint32_t our_time = getRTCClock()->getCurrentTime();
    if (our_time >= TIMESYNC_SANITY_CHECK_EPOCH) {
      TimeSyncMonitor::processAdvertisement(pkt, id, timestamp, parser.getName(), our_time);
    }
#endif

    uint8_t hash[MAX_HASH_SIZE];
    pkt->calculatePacketHash(hash);

    char pubkey[(PUB_KEY_SIZE * 2) + 1];
    char sender[(PUB_KEY_SIZE * 2) + 1];
    char strhash[MAX_HASH_SIZE * 2 + 1];

    mesh::Utils::toHex(pubkey, id.pub_key, PUB_KEY_SIZE);
    mesh::Utils::toHex(sender, self_id.pub_key, PUB_KEY_SIZE);
    mesh::Utils::toHex(strhash, hash, MAX_HASH_SIZE);

    JsonDocument doc;
    doc["version"] = 1;
    doc["type"] = "ADV";
    doc["reporter"] = sender;
    doc["hash"] = strhash;
    doc["snr"] = pkt->getSNR();
    doc["time"]["local"] = getRTCClock()->getCurrentTime();
    doc["time"]["sender"] = timestamp;
    doc["contact"]["new"] = is_new;
    doc["contact"]["type"] = parser.getType();
    doc["contact"]["feat1"] = parser.getFeat1();
    doc["contact"]["feat2"] = parser.getFeat2();
    doc["contact"]["flags"] = app_data[0];
    doc["contact"]["name"] = parser.getName();
    doc["contact"]["pubkey"] = pubkey;
    doc["contact"]["lat"] = parser.getIntLat();
    doc["contact"]["lon"] = parser.getIntLon();
    doc["message"]["path"] = getPath(pkt);
    messageQueue.push(doc);

    // Serial prints
    if (debugPrint()) {
      Serial.printf("ADVERT from -> %s\n", parser.getName());
      Serial.printf("  lat:       %.6f\n", parser.getIntLat() / 1000000.0);
      Serial.printf("  lon:       %.6f\n", parser.getIntLon() / 1000000.0);
    }
  }

  void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t* path) override {
    saveContacts();
  }

  String getPath(mesh::Packet* pkt) {
    String path = "";
    if (!pkt->isRouteDirect()) {
      char buf[4];
      for (size_t i = 0; i < pkt->path_len; i++) {
        sprintf(buf, "%02x", pkt->path[i]);
        if (i != 0) path += ",";
        path += buf;
      }
    }
    return path;
  }

  void onContactPathUpdated(const ContactInfo& contact) override {
    Serial.printf("PATH to: %s, path_len=%d\n", contact.name, (int32_t) contact.out_path_len);
    saveContacts();
  }

  ContactInfo* processAck(const uint8_t *data) override {
    if (memcmp(data, &expected_ack_crc, 4) == 0) {     // got an ACK from recipient
      Serial.printf("   Got ACK! (round trip: %d millis)\n", _ms->getMillis() - last_msg_sent);
      // NOTE: the same ACK can be received multiple times!
      expected_ack_crc = 0;  // reset our expected hash, now that we have received ACK
      return nullptr;  // Return nullptr instead of true
    }

    //uint32_t crc;
    //memcpy(&crc, data, 4);
    //MESH_DEBUG_PRINTLN("unknown ACK received: %08X (expected: %08X)", crc, expected_ack_crc);
    return nullptr;
  }

  void onMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // OPTIMIZED: Quick processing to prevent packet buffer overflow
    // Process critical operations first, then queue for background processing
    
    unsigned long msg_start = millis();
    uint8_t hash[MAX_HASH_SIZE];
    pkt->calculatePacketHash(hash);

    char pubkey[(PUB_KEY_SIZE * 2) + 1];
    char sender[(PUB_KEY_SIZE * 2) + 1];
    char strhash[MAX_HASH_SIZE * 2 + 1];

    mesh::Utils::toHex(pubkey, from.id.pub_key, PUB_KEY_SIZE);
    mesh::Utils::toHex(sender, self_id.pub_key, PUB_KEY_SIZE);
    mesh::Utils::toHex(strhash, hash, MAX_HASH_SIZE);

    JsonDocument doc;
    doc["version"] = 1;
    doc["type"] = "MSG";
    doc["reporter"] = sender;
    doc["hash"] = strhash;
    doc["snr"] = pkt->getSNR();
    doc["time"]["local"] = getRTCClock()->getCurrentTime();
    doc["time"]["sender"] = sender_timestamp;
    doc["contact"]["type"] = from.type;
    doc["contact"]["flags"] = from.flags;
    doc["contact"]["pubkey"] = pubkey;
    doc["contact"]["name"] = from.name;
    doc["contact"]["lat"] = from.gps_lat;
    doc["contact"]["lon"] = from.gps_lon;
    doc["message"]["text"] = text;
    doc["message"]["header"] = pkt->header;
    doc["message"]["path"] = getPath(pkt);
    
    // Queue message quickly (non-blocking)
    unsigned long queue_start = millis();
    messageQueue.push(doc);
    unsigned long queue_time = millis() - queue_start;
    
    unsigned long msg_time = millis() - msg_start;
    if (msg_time > 50) {
      Serial.printf("[MSG] Message processing took %lums (queue: %lums) - from %s\n", msg_time, queue_time, from.name);
    }

    // Check for ping message and send pong response
#ifdef PINGPONG_ENABLED
        if (PingPongHelper::processMessage(*this, from, pkt, sender_timestamp, text)) {
          return;
        }
#endif

#ifdef TIMESYNC_MONITOR_ENABLED
    // Check for timesync shame list command
    if (TimeSyncMonitor::processShameListCommand(*this, from, pkt, sender_timestamp, text)) {
      return;
    }
#endif

    // Special commands
    if (strcmp(text, "clock sync") == 0) {  // special text command
      setClock(sender_timestamp + 1, false);
    } else if (memcmp(text, "echo ", 5) == 0) {  // special text command
      const char* echo = &text[5];
      uint32_t est_timeout;
      last_msg_sent = _ms->getMillis();
      sendMessage(from, getRTCClock()->getCurrentTime(), 0, echo, expected_ack_crc, est_timeout);
    } else if (strcmp(text, "start ota") == 0) {  // Admin-only: Start OTA
      if (isAdmin(from.id.pub_key)) {
        Serial.printf("Authorized OTA request from %s\n", from.name);
        
        // Start OTA server on existing WiFi connection
        if (WiFi.status() == WL_CONNECTED) {
          char id[160];
          sprintf(id, "MeshCore Logger (%s %s)", __DATE__, __TIME__);
          AsyncWebServer* server = new AsyncWebServer(80);
          AsyncElegantOTA.setID(id);
          AsyncElegantOTA.begin(server);
          server->begin();
          
          // Send response with OTA URL
          char response[128];
          sprintf(response, "OTA started: http://%s/update", WiFi.localIP().toString().c_str());
          Serial.println(response);
          
          uint32_t est_timeout;
          last_msg_sent = _ms->getMillis();
          sendMessage(from, getRTCClock()->getCurrentTime(), 0, response, expected_ack_crc, est_timeout);
        } else {
          // WiFi not connected
          char response[] = "OTA failed: WiFi not connected";
          Serial.println(response);
          
          uint32_t est_timeout;
          last_msg_sent = _ms->getMillis();
          sendMessage(from, getRTCClock()->getCurrentTime(), 0, response, expected_ack_crc, est_timeout);
        }
      } else {
        char sender_pubkey_hex[PUB_KEY_SIZE * 2 + 1];
        mesh::Utils::toHex(sender_pubkey_hex, from.id.pub_key, PUB_KEY_SIZE);
        Serial.printf("Unauthorized OTA request from %s (%s)\n", from.name, sender_pubkey_hex);
        
        // Send rejection message
        char response[] = "Access denied: Not authorized";
        uint32_t est_timeout;
        last_msg_sent = _ms->getMillis();
        sendMessage(from, getRTCClock()->getCurrentTime(), 0, response, expected_ack_crc, est_timeout);
      }
    } else if (strcmp(text, "reboot") == 0) {  // Admin-only: Reboot device
      if (isAdmin(from.id.pub_key)) {
        Serial.printf("Authorized reboot request from %s\n", from.name);
        
        // Send acknowledgment before rebooting
        char response[] = "Rebooting now...";
        uint32_t est_timeout;
        last_msg_sent = _ms->getMillis();
        sendMessage(from, getRTCClock()->getCurrentTime(), 0, response, expected_ack_crc, est_timeout);
        
        // Give time for message to be sent
        delay(2000);
        
        Serial.println("Rebooting...");
        ESP.restart();
      } else {
        char sender_pubkey_hex[PUB_KEY_SIZE * 2 + 1];
        mesh::Utils::toHex(sender_pubkey_hex, from.id.pub_key, PUB_KEY_SIZE);
        Serial.printf("Unauthorized reboot request from %s (%s)\n", from.name, sender_pubkey_hex);
        
        // Send rejection message
        char response[] = "Access denied: Not authorized";
        uint32_t est_timeout;
        last_msg_sent = _ms->getMillis();
        sendMessage(from, getRTCClock()->getCurrentTime(), 0, response, expected_ack_crc, est_timeout);
      }
    }
  }

  void onCommandDataRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const char *text) override {
    // logmsg* m = new logmsg;
    // m->prio = "4";
    // m->time = sender_timestamp;
    // m->tag = from.name;
    // m->msg = "Command: ";
    // m->msg += text;
    // pendingMsgs.push_back(m);
  }
  
  void onSignedMessageRecv(const ContactInfo& from, mesh::Packet* pkt, uint32_t sender_timestamp, const uint8_t *sender_prefix, const char *text) override {
  }

  void onChannelMessageRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t timestamp, const char *text) override {
    // Add debug logging
    MESH_DEBUG_PRINTLN("[CHANNEL] Received: %s", text);
    
    uint8_t hash[MAX_HASH_SIZE];
    pkt->calculatePacketHash(hash);

    char chhash[(PUB_KEY_SIZE * 2) + 1];
    char sender[(PUB_KEY_SIZE * 2) + 1];
    char strhash[MAX_HASH_SIZE * 2 + 1];

    mesh::Utils::toHex(chhash, channel.hash, PATH_HASH_SIZE);
    mesh::Utils::toHex(sender, self_id.pub_key, PUB_KEY_SIZE);
    mesh::Utils::toHex(strhash, hash, MAX_HASH_SIZE);

    // Find channel name by matching hash
    const char* channel_name = "unknown";
#ifdef MAX_GROUP_CHANNELS
    ChannelDetails ch;
    for (int i = 0; i < MAX_GROUP_CHANNELS; i++) {
      if (getChannel(i, ch)) {
        if (memcmp(ch.channel.hash, channel.hash, PATH_HASH_SIZE) == 0) {
          channel_name = ch.name;
          break;
        }
      }
    }
#endif

    JsonDocument doc;
    doc["version"] = 1;
    doc["type"] = "PUB";
    doc["reporter"] = sender;
    doc["hash"] = strhash;
    doc["snr"] = pkt->getSNR();
    doc["time"]["local"] = getRTCClock()->getCurrentTime();
    doc["time"]["sender"] = timestamp;
    doc["message"]["text"] = text;
    doc["message"]["header"] = pkt->header;
    doc["message"]["path"] = getPath(pkt);
    doc["channel"]["hash"] = chhash;
    doc["channel"]["name"] = channel_name;

    messageQueue.push(doc);

    if (pkt->isRouteDirect()) {
      MESH_DEBUG_PRINTLN("PUBLIC CHANNEL MSG -> (Direct!)");
    } else {
      MESH_DEBUG_PRINTLN("PUBLIC CHANNEL MSG -> (Flood) hops %d", pkt->path_len);
    }

    MESH_DEBUG_PRINTLN("   %s", text);

        // Check for ping message in channel and send pong response (only #ping channel)
#ifdef PINGPONG_ENABLED
        // Only respond to ping in the #ping channel (compare hash directly)
        bool isPingChannel = false;
        if (_ping_channel) {
            isPingChannel = (memcmp(channel.hash, _ping_channel->channel.hash, PATH_HASH_SIZE) == 0);
        }
        
        if (PingPongHelper::isPingMessage(text) && isPingChannel) {
          // Extract sender name from channel message format: "<sender>: <msg>"
          char sender_name[32] = "Unknown";
          const char* colon_pos = strchr(text, ':');
          if (colon_pos && colon_pos > text) {
            size_t sender_len = colon_pos - text;
            if (sender_len < sizeof(sender_name)) {
              strncpy(sender_name, text, sender_len);
              sender_name[sender_len] = '\0';
            }
          }
          
          MESH_DEBUG_PRINTLN("[PING-CH] Processing ping from: %s", sender_name);
          
          // Per-sender 15-second cooldown (allows multiple people to ping simultaneously)
          bool canRespond = PingPongHelper::canRespondToChannelSender(sender_name, 15000);
          
          if (canRespond) {
            MESH_DEBUG_PRINTLN("[PING-CH] Can respond to %s (not in cooldown)", sender_name);
            
            // Get RSSI from radio
            float rssi = 0.0;
            if (getRadio()) {
              rssi = getRadio()->getLastRSSI();
            }
            
            // Generate pong response
            char response[256];
            char router_ids_buffer[256];
            uint8_t hop_count = pkt->path_len;
            
            // Extract path info
            if (PingPongHelper::extractPathInfo(pkt, hop_count, router_ids_buffer, sizeof(router_ids_buffer))) {
              if (PingPongHelper::generatePongResponse(sender_name, hop_count, router_ids_buffer, 
                                                       pkt->getSNR(), rssi, true, response, sizeof(response))) {
                // Create channel message packet for retry system
                uint8_t temp[5+MAX_TEXT_LEN+32];
                uint32_t timestamp = getRTCClock()->getCurrentTime();
                memcpy(temp, &timestamp, 4);
                temp[4] = 0;  // TXT_TYPE_PLAIN

                // Format: <sender_name>: <response>
                sprintf((char *) &temp[5], "%s: %s", _prefs.node_name, response);
                temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

                int len = strlen((char *) &temp[5]);
                auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + len);
                if (!pkt) {
                  MESH_DEBUG_PRINTLN("[PING-CH] ERROR: Failed to create group datagram (packet pool full?)");
                  return;
                }
                
                // Schedule delayed channel response (randomized 6-8s allows network storm to settle
                // and prevents multiple bots from responding simultaneously)
                uint32_t delay = getRNG()->nextInt(6000, 8001);  // 6000-8000 ms
                MESH_DEBUG_PRINTLN("[PING-CH] Scheduling response to %s with %dms delay", sender_name, delay);
                
                // Send with retry system
                sendFlood(pkt, delay);
                
                // Track packet AFTER sending (not before) to avoid false repetition detection
                _tables->trackSentPacket(pkt);
                scheduleRetry(pkt, delay + getRNG()->nextInt(2000, 4001));  // Schedule retry check
              } else {
                MESH_DEBUG_PRINTLN("[PING-CH] ERROR: Failed to generate pong response");
              }
            } else {
              MESH_DEBUG_PRINTLN("[PING-CH] ERROR: Failed to extract path info");
            }
          } else {
            MESH_DEBUG_PRINTLN("[PING-CH] Cannot respond to %s (in cooldown)", sender_name);
          }
        }
#endif
  }
  
  uint8_t onContactRequest(const ContactInfo& contact, uint32_t sender_timestamp, const uint8_t* data, uint8_t len, uint8_t* reply) override {
    return 0;  // unknown
  }

  void onContactResponse(const ContactInfo& contact, const uint8_t* data, uint8_t len) override {
    // not supported
  }

  uint32_t calcFloodTimeoutMillisFor(uint32_t pkt_airtime_millis) const override {
    return SEND_TIMEOUT_BASE_MILLIS + (FLOOD_SEND_TIMEOUT_FACTOR * pkt_airtime_millis);
  }
  
  uint32_t calcDirectTimeoutMillisFor(uint32_t pkt_airtime_millis, uint8_t path_len) const override {
    return SEND_TIMEOUT_BASE_MILLIS + 
         ( (pkt_airtime_millis*DIRECT_SEND_PERHOP_FACTOR + DIRECT_SEND_PERHOP_EXTRA_MILLIS) * (path_len + 1));
  }

  void onSendTimeout() override {
    Serial.println("   ERROR: timed out, no ACK.");
  }

public:
  MyMesh(mesh::Radio& radio, StdRNG& rng, mesh::RTCClock& rtc, LoggerMeshTables& tables)
     : BaseChatMesh(radio, *new ArduinoMillis(), rng, rtc, *new StaticPoolPacketManager(32), tables)
  {
    // defaults
    memset(&_prefs, 0, sizeof(_prefs));
    memset(&_wifi, 0, sizeof(_wifi));
    memset(&_logp, 0, sizeof(_logp));
    _prefs.airtime_factor = 2.0;    // one third
    strcpy(_prefs.node_name, "NONAME");
    _prefs.freq = LORA_FREQ;
    _prefs.tx_power_dbm = LORA_TX_POWER;
    _prefs.bw = LORA_BW;
    _prefs.sf = LORA_SF;
    _prefs.cr = LORA_CR;

    command[0] = 0;
    curr_recipient = NULL;
    _tables = &tables;
    
    // Initialize retry system
    memset(pending_retries, 0, sizeof(pending_retries));
  }

  float getFreqPref() const { return _prefs.freq; }
  uint8_t getTxPowerPref() const { return _prefs.tx_power_dbm; }
  float getBwPref() const { return _prefs.bw; }
  uint8_t getSfPref() const { return _prefs.sf; }
  uint8_t getCrPref() const { return _prefs.cr; }

  const uint8_t* getPubKey() {
    return self_id.pub_key;
  }

  void begin(FILESYSTEM& fs) {
    _fs = &fs;

    BaseChatMesh::begin();

    IdentityStore store(fs, "/identity");

    if (!store.load("_main", self_id, _prefs.node_name, sizeof(_prefs.node_name))) {  // legacy: node_name was from identity file
      // Need way to get some entropy to seed RNG
      Serial.println("Press ENTER to generate key:");
      char c = 0;
      while (c != '\n') {   // wait for ENTER to be pressed
        if (Serial.available()) c = Serial.read();
      }
      Serial.println("generating key...");
      ((StdRNG *)getRNG())->begin(millis());

      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      Serial.println("done.");
      store.save("_main", self_id);
    }

    // load persisted prefs
    if (_fs->exists("/node_prefs")) {
      File file = _fs->open("/node_prefs");
      if (file) {
        file.read((uint8_t *) &_prefs, sizeof(_prefs));
        file.close();
      }
    }

    // load wifi prefs
    if (_fs->exists("/wifi_prefs")) {
      File file = _fs->open("/wifi_prefs");
      if (file) {
        file.read((uint8_t *) &_wifi, sizeof(_wifi));
        file.close();
      }
    }

    // load wifi prefs
    if (_fs->exists("/log_prefs")) {
      File file = _fs->open("/log_prefs");
      if (file) {
        file.read((uint8_t *) &_logp, sizeof(_logp));
        file.close();

        if (_logp.version == 0) {
          _logp.version = 1;
          _logp.selfreport = 15 * 60; // 15 min default
          saveLogPrefs();
        }
      }
    }

    loadContacts();
    loadChannels();  // Load existing channels from flash first
    loadAdmins();    // Load authorized admin keys
    _public = addChannel("Public", PUBLIC_GROUP_PSK); // pre-configure Andy's public channel

    // Add additional channels
    addChannel("#hungary", "0q1+QAm3J/tO5cH/UWlOXg==");  // #hungary channel
    addChannel("#austria", "+qpe8BCBIi4xmoIFNXMh9A==");  // #austria channel (hex: faaa5ef01081222e319a8205357321f)
    addChannel("#slovakia", "VQuKlUbVYYMQB0/boDaPmA==");  // #slovakia channel (hex: 550b8a9546d5618310074fdba0368f9)
    _ping_channel = addChannel("#ping", "PK4W/QZ7qcMqmL4i6bmFJQ==");  // #ping channel

    // Save channels to flash so they persist
    saveChannels();
    
#ifdef PINGPONG_ENABLED
    // Initialize pingpong helper
    PingPongHelper::begin();
#endif

#ifdef TIMESYNC_MONITOR_ENABLED
    // Initialize time sync monitor with filesystem persistence
    TimeSyncMonitor::begin(_fs, "/timesync_good");
    if (_public) {
      TimeSyncMonitor::setPublicChannel(&_public->channel);
    }
#endif
    
    // Schedule first NTP sync for 10 seconds after boot
    ntpNext = millis() + ntpFirstSyncDelay;

    toggleWiFi(true);
  }

  void toggleWiFi(bool enable) {
    if (strlen(_wifi.ssid) < 1) {
      Serial.println("WiFi: SSID not set");
      return;
    }
    if (enable) {
      Serial.printf("WiFi: Conencting to %s\n", _wifi.ssid);
      WiFi.mode(WIFI_STA);
      WiFi.begin(_wifi.ssid, _wifi.password);
    } else {
      Serial.println("WiFi: Disconencting");
      WiFi.disconnect();
    }
  }

  void savePrefs() {
    File file = _fs->open("/node_prefs", "w", true);
    if (file) {
      file.write((const uint8_t *)&_prefs, sizeof(_prefs));
      file.close();
    }
  }

  void saveWiFiPrefs() {
    File file = _fs->open("/wifi_prefs", "w", true);
    if (file) {
      file.write((const uint8_t *)&_wifi, sizeof(_wifi));
      file.close();
    }

    toggleWiFi(false);
    toggleWiFi(true);
  }

  void saveLogPrefs() {
    File file = _fs->open("/log_prefs", "w", true);
    if (file) {
      file.write((const uint8_t *)&_logp, sizeof(_logp));
      file.close();
    }
  }

  void showWelcome() {
    Serial.println("===== MeshCore Chat Terminal =====");
    Serial.println();
    Serial.printf("WELCOME  %s\n", _prefs.node_name);
    mesh::Utils::printHex(Serial, self_id.pub_key, PUB_KEY_SIZE);
    Serial.println();
    Serial.println("   (enter 'help' for basic commands)");
    Serial.println();
  }

  void sendSelfAdvert(int delay_millis) {
    auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
    if (pkt) {
      sendZeroHop(pkt, delay_millis);
    }
  }

  // ContactVisitor
  void onContactVisit(const ContactInfo& contact) override {
    Serial.printf("   %s - ", contact.name);
    char tmp[40];
    int32_t secs = contact.last_advert_timestamp - getRTCClock()->getCurrentTime();
    AdvertTimeHelper::formatRelativeTimeDiff(tmp, secs, false);
    Serial.println(tmp);
  }

  void handleCommand(const char* command) {
    while (*command == ' ') command++;  // skip leading spaces

    if (memcmp(command, "send ", 5) == 0) {
      if (curr_recipient) {
        const char *text = &command[5];
        uint32_t est_timeout;

        int result = sendMessage(*curr_recipient, getRTCClock()->getCurrentTime(), 0, text, expected_ack_crc, est_timeout);
        if (result == MSG_SEND_FAILED) {
          Serial.println("   ERROR: unable to send.");
        } else {
          last_msg_sent = _ms->getMillis();
          Serial.printf("   (message sent - %s)\n", result == MSG_SEND_SENT_FLOOD ? "FLOOD" : "DIRECT");
        }
      } else {
        Serial.println("   ERROR: no recipient selected (use 'to' cmd).");
      }
    } else if (memcmp(command, "public ", 7) == 0) {  // send GroupChannel msg
      uint8_t temp[5+MAX_TEXT_LEN+32];
      uint32_t timestamp = getRTCClock()->getCurrentTime();
      memcpy(temp, &timestamp, 4);   // mostly an extra blob to help make packet_hash unique
      temp[4] = 0;  // attempt and flags

      sprintf((char *) &temp[5], "%s: %s", _prefs.node_name, &command[7]);  // <sender>: <msg>
      temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

      int len = strlen((char *) &temp[5]);
      auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, _public->channel, temp, 5 + len);
      if (pkt) {
        sendFlood(pkt);
        Serial.println("   Sent.");
      } else {
        Serial.println("   ERROR: unable to send");
      }
    } else if (memcmp(command, "list", 4) == 0) {  // show Contact list, by most recent
      int n = 0;
      if (command[4] == ' ') {  // optional param, last 'N'
        n = atoi(&command[5]);
      }
      scanRecentContacts(n, this);
    } else if (strcmp(command, "clock") == 0) {    // show current time
      uint32_t now = getRTCClock()->getCurrentTime();
      DateTime dt = DateTime(now);
      Serial.printf(   "%02d:%02d - %d/%d/%d UTC\n", dt.hour(), dt.minute(), dt.day(), dt.month(), dt.year());
    } else if (memcmp(command, "time ", 5) == 0) {  // set time (to epoch seconds)
      const char* config = &command[5];
      if (memcmp(config, "ntp", 3) == 0) {
        ntpSynced = false;
      } else {
        uint32_t secs = _atoi(config);
        setClock(secs, false);
      }
    } else if (memcmp(command, "to ", 3) == 0) {  // set current recipient
      curr_recipient = searchContactsByPrefix(&command[3]);
      if (curr_recipient) {
        Serial.printf("   Recipient %s now selected.\n", curr_recipient->name);
      } else {
        Serial.println("   Error: Name prefix not found.");
      }
    } else if (strcmp(command, "to") == 0) {    // show current recipient
      if (curr_recipient) {
         Serial.printf("   Current: %s\n", curr_recipient->name);
      } else {
         Serial.println("   Err: no recipient selected");
      }
    } else if (strcmp(command, "get radio") == 0) {
      Serial.println("Current Radio Settings:");
      Serial.printf("   Frequency: %.3f MHz\n", _prefs.freq);
      Serial.printf("   Bandwidth: %.1f kHz\n", _prefs.bw);
      Serial.printf("   Spreading Factor: %d\n", _prefs.sf);
      Serial.printf("   Coding Rate: %d\n", _prefs.cr);
      Serial.printf("   TX Power: %d dBm\n", _prefs.tx_power_dbm);
    } else if (strcmp(command, "advert") == 0) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        sendZeroHop(pkt);
        Serial.println("   (advert sent, zero hop).");
      } else {
        Serial.println("   ERR: unable to send");
      }
    } else if (strcmp(command, "flood") == 0) {
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        sendFlood(pkt);
        Serial.println("   (advert sent, flood).");
      } else {
        Serial.println("   ERR: unable to send");
      }
    } else if (strcmp(command, "reset path") == 0) {
      if (curr_recipient) {
        resetPathTo(*curr_recipient);
        saveContacts();
        Serial.println("   Done.");
      }
    } else if (strcmp(command, "clear contacts") == 0) {
      int count_before = getNumContacts();
      resetContacts();
      curr_recipient = nullptr;
      if (_fs->exists("/contacts")) {
        _fs->remove("/contacts");
      }
#ifdef TIMESYNC_MONITOR_ENABLED
      if (_fs->exists("/timesync_good")) {
        _fs->remove("/timesync_good");
      }
#endif
      Serial.printf("   All contacts cleared. (%d contacts removed)\n", count_before);
    } else if (memcmp(command, "card", 4) == 0) {
      Serial.printf("Hello %s\n", _prefs.node_name);
      auto pkt = createSelfAdvert(_prefs.node_name, _prefs.node_lat, _prefs.node_lon);
      if (pkt) {
        uint8_t len =  pkt->writeTo(tmp_buf);
        releasePacket(pkt);  // undo the obtainNewPacket()

        mesh::Utils::toHex(hex_buf, tmp_buf, len);
        Serial.println("Your MeshCore biz card:");
        Serial.print("meshcore://"); Serial.println(hex_buf);
        Serial.println();
      } else {
        Serial.println("  Error");
      }
    } else if (memcmp(command, "import ", 7) == 0) {
      importCard(&command[7]);
    } else if (memcmp(command, "channel ", 8) == 0) {
      const char* method = &command[8];
      if (memcmp(method, "add ", 4) == 0) {
        // Parse: "channel add <name> <hex_psk>"
        // Find the space between name and hex_psk
        const char* args = &method[4];
        const char* space_pos = strchr(args, ' ');
        if (!space_pos) {
          Serial.println("  ERROR: Usage: channel add <name> <hex_psk>");
        } else {
          // Extract name (everything before space)
          int name_len = space_pos - args;
          if (name_len <= 0 || name_len >= 32) {
            Serial.println("  ERROR: Invalid channel name length");
          } else {
            char channel_name[32];
            memcpy(channel_name, args, name_len);
            channel_name[name_len] = '\0';
            
            // Extract hex PSK (everything after space)
            const char* hex_psk = space_pos + 1;
            while (*hex_psk == ' ') hex_psk++;  // Skip leading spaces
            
            // Convert hex PSK to base64
            char base64_psk[45];  // Base64 encoded 16 or 32 bytes + null terminator
            if (hexPskToBase64(hex_psk, base64_psk, sizeof(base64_psk))) {
              ChannelDetails* ch = addChannel(channel_name, base64_psk);
              if (ch) {
                saveChannels();
                Serial.printf("  Channel '%s' added\n", channel_name);
              } else {
                Serial.println("  ERROR: Failed to add channel (max channels reached?)");
              }
            } else {
              Serial.println("  ERROR: Invalid hex PSK format (must be 32 or 64 hex characters)");
            }
          }
        }
      } else if (memcmp(method, "delete ", 7) == 0) {
        // Parse: "channel delete <id>"
        const char* id_str = &method[7];
        while (*id_str == ' ') id_str++;  // Skip leading spaces
        
        int idx = atoi(id_str);
        if (removeChannelByIdx(idx)) {
          saveChannels();
          Serial.printf("  Channel at index %d deleted\n", idx);
        } else {
          Serial.printf("  ERROR: Channel at index %d not found\n", idx);
        }
      } else if (memcmp(method, "delete", 6) == 0) {
        deleteChannels();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(method, "ls", 2) == 0) {
        uint8_t channel_idx = 0;
        ChannelDetails ch;
        uint8_t unused[4];
        memset(unused, 0, 4);

        Serial.println("Channels:");
        Serial.println("  ID  Name");
        Serial.println("  --- ----");
        while (getChannel(channel_idx, ch)) {
          char hash_str[3];
          mesh::Utils::toHex(hash_str, ch.channel.hash, PATH_HASH_SIZE);
          hash_str[2] = '\0';
          Serial.printf("  %s  %s (idx: %d)\n", hash_str, ch.name, channel_idx);
          channel_idx++;
        }
        Serial.println();
      } else {
        Serial.println("  Invalid option");
      }
    } else if (memcmp(command, "set ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "af ", 3) == 0) {
        _prefs.airtime_factor = atof(&config[3]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "name ", 5) == 0) {
        StrHelper::strncpy(_prefs.node_name, &config[5], sizeof(_prefs.node_name));
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lat ", 4) == 0) {
        _prefs.node_lat = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "lon ", 4) == 0) {
        _prefs.node_lon = atof(&config[4]);
        savePrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "tx ", 3) == 0) {
        _prefs.tx_power_dbm = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "freq ", 5) == 0) {
        _prefs.freq = atof(&config[5]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "bw ", 3) == 0) {
        _prefs.bw = atof(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "sf ", 3) == 0) {
        _prefs.sf = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else if (memcmp(config, "cr ", 3) == 0) {
        _prefs.cr = atoi(&config[3]);
        savePrefs();
        Serial.println("  OK - reboot to apply");
      } else {
        Serial.printf("  ERROR: unknown config: %s\n", config);
      }
    } else if (memcmp(command, "ver", 3) == 0) {
      Serial.println(FIRMWARE_VER_TEXT);
    } else if (memcmp(command, "wifi ", 5) == 0) {
      const char* config = &command[5];
      if (memcmp(config, "ssid ", 5) == 0) {
        StrHelper::strncpy(_wifi.ssid, &config[5], sizeof(_wifi.ssid));
        saveWiFiPrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "password ", 9) == 0) {
        StrHelper::strncpy(_wifi.password, &config[9], sizeof(_wifi.password));
        saveWiFiPrefs();
        Serial.println("  OK");
      } else {
        Serial.printf("  WiFi Conencted: %u\n", WiFi.status() == WL_CONNECTED);
      }
    } else if (memcmp(command, "log ", 4) == 0) {
      const char* config = &command[4];
      if (memcmp(config, "url ", 4) == 0) {
        StrHelper::strncpy(_logp.url, &config[4], sizeof(_logp.url));
        saveLogPrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "auth ", 5) == 0) {
        StrHelper::strncpy(_logp.auth, &config[5], sizeof(_logp.auth));
        saveLogPrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "report ", 7) == 0) {
        _logp.selfreport = atoi(&config[7]);
        saveLogPrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "raw ", 4) == 0) {
        if (config[4] == 'y') {
          _logp.doraw = 1;
        } else {
          _logp.doraw = 0;
        }
        saveLogPrefs();
        Serial.println("  OK");
      } else if (memcmp(config, "fwd ", 4) == 0) {
        if (config[4] == 'y') {
          _logp.dofwd = 1;
        } else {
          _logp.dofwd = 0;
        }
        saveLogPrefs();
        Serial.println("  OK");
      } else {
        char sender[(PUB_KEY_SIZE * 2) + 1];
        mesh::Utils::toHex(sender, self_id.pub_key, PUB_KEY_SIZE);
        Serial.printf("  Log url:     %s\n", _logp.url);
        Serial.printf("  Self-report: %u\n", _logp.selfreport);
        Serial.printf("  Pub Key:     %s\n", sender);
        Serial.printf("  Raw:         %u\n", _logp.doraw);
      }
    } else if (memcmp(command, "debug ", 6) == 0) { 
      if (command[6] == 'y') {
        m_debugPrint = true;
      } else {
        m_debugPrint = false;
      }
      Serial.printf("  Debug print: %u\n", m_debugPrint);
    } else if (memcmp(command, "admin ", 6) == 0) {
      const char* subcmd = &command[6];
      if (memcmp(subcmd, "add ", 4) == 0) {
        const char* hex_key = &subcmd[4];
        // Remove any whitespace
        while (*hex_key == ' ') hex_key++;
        
        int len = strlen(hex_key);
        if (len == PUB_KEY_SIZE * 2) {
          uint8_t pub_key[PUB_KEY_SIZE];
          if (mesh::Utils::fromHex(pub_key, PUB_KEY_SIZE, hex_key)) {
            if (addAdmin(pub_key)) {
              Serial.println("  Admin added successfully");
              char hex_buf[PUB_KEY_SIZE * 2 + 1];
              mesh::Utils::toHex(hex_buf, pub_key, PUB_KEY_SIZE);
              Serial.printf("  %s\n", hex_buf);
            } else {
              Serial.println("  Error: Admin already exists or list is full");
            }
          } else {
            Serial.println("  Error: Invalid hex format");
          }
        } else {
          Serial.printf("  Error: Wrong length (expected %d hex chars, got %d)\n", PUB_KEY_SIZE * 2, len);
        }
      } else if (memcmp(subcmd, "ls", 2) == 0) {
        Serial.printf("Authorized Admins (%u/%u):\n", _admin.count, MAX_ADMINS);
        for (uint16_t i = 0; i < _admin.count; i++) {
          char hex_buf[PUB_KEY_SIZE * 2 + 1];
          mesh::Utils::toHex(hex_buf, _admin.admin_keys[i], PUB_KEY_SIZE);
          Serial.printf("  [%d] %s\n", i, hex_buf);
        }
      } else if (memcmp(subcmd, "clear", 5) == 0) {
        _admin.count = 0;
        memset(_admin.admin_keys, 0, sizeof(_admin.admin_keys));
        saveAdmins();
        Serial.println("  All admins cleared");
      } else {
        Serial.println("  Usage: admin {add|ls|clear}");
      }
    } else if (memcmp(command, "regeneratekey", 13) == 0) {
      IdentityStore store(*_fs, "/identity");
      Serial.println("generating key...");
      ((StdRNG *)getRNG())->begin(millis());
    
      self_id = mesh::LocalIdentity(getRNG());  // create new random identity
      int count = 0;
      while (count < 10 && (self_id.pub_key[0] == 0x00 || self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
        self_id = mesh::LocalIdentity(getRNG()); count++;
      }
      Serial.println("done.");
      store.save("_main", self_id);
    } else if (memcmp(command, "reboot", 6) == 0) {
      Serial.println("Rebooting...");
      ESP.restart();
    } else if (memcmp(command, "start ota", 9) == 0) {
      char id[160];
      sprintf(id, "MeshCore Logger (%s %s)", __DATE__, __TIME__);
      AsyncWebServer* server = new AsyncWebServer(80);
      AsyncElegantOTA.setID(id);
      AsyncElegantOTA.begin(server);    // Start ElegantOTA
      server->begin();
      Serial.print("  Go to http://");
      Serial.print(WiFi.localIP());
      Serial.println("/update");
    } else if (memcmp(command, "help", 4) == 0) {
      Serial.println("Commands:");
      Serial.println("   set {name|lat|lon|freq|tx|af|bw|sf|cr} {value}");
      Serial.println("   get radio");
      Serial.println("   card");
      Serial.println("   import {biz card}");
      Serial.println("   clock");
      Serial.println("   time {epoch-seconds|ntp}>");
      Serial.println("   list {n}");
      Serial.println("   to <recipient name or prefix>");
      Serial.println("   to");
      Serial.println("   send <text>");
      Serial.println("   advert");
      Serial.println("   flood");
      Serial.println("   reset path");
      Serial.println("   public <text>");
      Serial.println("   wifi {ssid|password} {value}");
      Serial.println("   log {url|auth|report|raw} {value}");
      Serial.println("   channel {add|ls} {value}");
      Serial.println("   admin {add|ls|clear} {pubkey}");
      Serial.println("   start ota");
    } else {
      Serial.print("   ERROR: unknown command: "); Serial.println(command);
    }
  }

  void loop() {
    BaseChatMesh::loop();
    
    // Process retry system for channel messages
    processRetries();

    int len = strlen(command);
    while (Serial.available() && len < sizeof(command)-1) {
      char c = Serial.read();
      if (c == 0x08) { // backspace
        if (len > 0) {
          command[len - 1] = 0;
          len--;
          Serial.print(c);
          Serial.print(' ');
        }
      } else if (c != '\n') {
        command[len++] = c;
        command[len] = 0;
      }
      Serial.print(c);
    }
    if (len == sizeof(command)-1) {  // command buffer full
      command[sizeof(command)-1] = '\r';
    }

    if (len > 0 && command[len - 1] == '\r') {  // received complete line
      command[len - 1] = 0;  // replace newline with C string null terminator

      handleCommand(command);
      command[0] = 0;  // reset command buffer
    }
  }
};

StdRNG fast_rng;
LoggerMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, *new VolatileRTCClock(), tables); // TODO: test with 'rtc_clock' in target.cpp

void halt() {
  while (1) ;
}

void WiFiTaskCode(void * pvParameters) {
  static bool connected = false;
  static bool sendsys   = false;
  static unsigned sendFailures = 0;
  static unsigned long lastConencted = 0;
  static unsigned long nextReport = 30000;

  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  WiFiClientSecure* client = new WiFiClientSecure;
  client->setInsecure();

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      sendsys = false;
      lastConencted = millis();

      // Check if it's time to sync (use current millis, not lastConencted)
      if (millis() >= ntpNext) {
        ntpSynced = false;
      }

      if (!ntpSynced) {
        Serial.println("[WiFi] Starting NTP sync...");
        unsigned long ntp_start = millis();
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        
        // Wait for NTP to complete (configTime is non-blocking on ESP32)
        task_sleep(2000);  // Wait 2 seconds for NTP response
        unsigned long ntp_time = millis() - ntp_start;
        Serial.printf("[WiFi] NTP sync took %lums\n", ntp_time);
        
        unsigned time = getTimestamp();
        if (time > 1735689600) {  // Valid time (after Jan 1, 2025)
          the_mesh.setClock(time, true);
          ntpSyncCount++;
          
          // Set next sync interval based on sync count
          if (ntpSyncCount == 1) {
            // After first sync, wait 5 minutes for second sync
            ntpNext = millis() + ntpSecondSyncInterval;
          } else {
            // After second sync and beyond, wait 2 hours
            ntpNext = millis() + ntpRegularSyncInterval;
          }
        }
        // If failed, ntpSynced stays false and will retry next loop iteration
      }

      if (the_mesh.getLogPrefs()->selfreport > 0 && millis() > nextReport) {
        char sender[(PUB_KEY_SIZE * 2) + 1];
        mesh::Utils::toHex(sender, the_mesh.getPubKey(), PUB_KEY_SIZE);
  
        JsonDocument doc;
        doc["version"] = 1;
        doc["type"] = "SYS";
        doc["reporter"] = sender;
        doc["time"]["local"] = getTimestamp(); //()->getCurrentTime();
        doc["sys"]["type"] = "status";
        doc["sys"]["heap_total"] = ESP.getHeapSize();
        doc["sys"]["heap_free"] = ESP.getFreeHeap();
        doc["sys"]["rssi"] = WiFi.RSSI();
        doc["sys"]["uptime"] = millis();
        doc["contact"]["new"] = false;
        doc["contact"]["type"] = ADV_TYPE_CHAT;
        doc["contact"]["flags"] = 0;
        doc["contact"]["name"] = the_mesh.getNodePrefs()->node_name;
        doc["contact"]["pubkey"] = sender;
        doc["contact"]["lat"] = the_mesh.getNodePrefs()->node_lat;
        doc["contact"]["lon"] = the_mesh.getNodePrefs()->node_lon;
        messageQueue.push(doc);
        if (the_mesh.getLogPrefs()->selfreport != -1) {
          nextReport = millis() + (the_mesh.getLogPrefs()->selfreport * 1000);
        }
      }

      if (messageQueue.size() > 0) {
        String auth = "Bearer ";
        auth += the_mesh.getLogPrefs()->auth;

        char *ptr = messageQueue.front();
        if (ptr != nullptr) {
          if (the_mesh.debugPrint()) {
            Serial.printf("Queue peek %s\n", ptr);
          }

          if (memcmp(the_mesh.getLogPrefs()->url, "http", 4) != 0) {
            Serial.println("Url not set.");
            messageQueue.pop();
            delete[] ptr;
          }

          bool sent = false;

          // WiFi send
          HTTPClient https;
          unsigned long http_start = millis();

          if (https.begin(*client, the_mesh.getLogPrefs()->url)) {  // HTTPS connection
            https.addHeader("Content-Type", "application/json");

            if (auth.length() > 7) {
              https.addHeader("Authorization", auth);
            }

            if (the_mesh.debugPrint()) {
              Serial.println("[HTTP] Post data");
            }
            int httpResponseCode = https.POST(ptr);
            unsigned long http_time = millis() - http_start;
        
            if (httpResponseCode > 0) {
              String response = https.getString();
              Serial.printf("[HTTP] POST: %d (took %lums)\n", httpResponseCode, http_time);
              sent = true;
            } else {
              Serial.printf("[HTTP] ERROR: %d (took %lums)\n", httpResponseCode, http_time);
              ++sendFailures;
            }
        
            https.end();
          } else {
            ++sendFailures;
            Serial.println("[HTTP] Unable to connect");
          }

          if (sent) {
            unsigned bef = ESP.getFreeHeap();
            sendFailures = 0;
            messageQueue.pop();
            delete[] ptr;
            unsigned aft = ESP.getFreeHeap();
            if (the_mesh.debugPrint()) {
              Serial.printf("free mem: %u -> %u >> %d\n",bef,aft,bef-aft);
            }
          }
        }
      }
    } else if (connected && (millis() > (lastConencted + 5000) || sendFailures > 5)) {
      connected = false;
      sendFailures = 0;
      char sender[(PUB_KEY_SIZE * 2) + 1];
      mesh::Utils::toHex(sender, the_mesh.getPubKey(), PUB_KEY_SIZE);

      Serial.println("Reconenct wifi...");

      // if WiFi is down, try reconnecting
      if (!sendsys) {
        JsonDocument doc;
        doc["version"] = 1;
        doc["type"] = "SYS";
        doc["reporter"] = sender;
        doc["time"]["local"] = getTimestamp(); //()->getCurrentTime();
        doc["sys"]["type"] = "wifi";
        doc["sys"]["message"] = "Reconnecting to WiFi...";
        doc["sys"]["heap_total"] = ESP.getHeapSize();
        doc["sys"]["heap_free"] = ESP.getFreeHeap();
        doc["sys"]["uptime"] = millis();
        messageQueue.push(doc);
        sendsys = true;
      }
  
      WiFi.disconnect();
      WiFi.reconnect();
      lastConencted = millis();
    }

    task_sleep(1000);
  }
}

void startWifiTask(int core) {
  xTaskCreatePinnedToCore(
    WiFiTaskCode,   /* Task function. */
      "WiFiTask",     /* name of task. */
      10000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &WiFiTask,      /* Task handle to keep track of created task */
      core);          /* pin task to core */
}

void setup() {
  Serial.begin(115200);

  delay(1000);

  board.begin();

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  SPIFFS.begin(true);
  the_mesh.begin(SPIFFS);

  radio_set_params(the_mesh.getFreqPref(), the_mesh.getBwPref(), the_mesh.getSfPref(), the_mesh.getCrPref());
  radio_set_tx_power(the_mesh.getTxPowerPref());

  the_mesh.showWelcome();

  // send out initial Advertisement to the mesh
  the_mesh.sendSelfAdvert(1200);   // add slight delay

  int core = xPortGetCoreID() == 1 ? 0 : 1;
  startWifiTask(core);
}

void loop() {
  static unsigned long last_loop_time = 0;
  static unsigned long max_loop_time = 0;
  static unsigned long last_pool_check = 0;
  unsigned long loop_start = millis();
  
  unsigned long mesh_start = millis();
  the_mesh.loop();
  unsigned long mesh_time = millis() - mesh_start;
  
  unsigned long timesync_start = millis();
#ifdef TIMESYNC_MONITOR_ENABLED
  TimeSyncMonitor::checkAndSendDailyReport(the_mesh, the_mesh.getRTCClock()->getCurrentTime(), the_mesh.getNodePrefs()->node_name);
  TimeSyncMonitor::processDelayedResponses();
  TimeSyncMonitor::processPendingSavesAsync();  // Use non-blocking version
#endif
  unsigned long timesync_time = millis() - timesync_start;

  // Monitor packet pool status every 30 seconds
  if (millis() - last_pool_check > 30000) {
    MESH_DEBUG_PRINTLN("[DEBUG] Queue: %u msgs", messageQueue.size());
    last_pool_check = millis();
  }

  // Monitor loop timing to detect blocking issues
  unsigned long loop_time = millis() - loop_start;
  if (loop_time > max_loop_time) {
    max_loop_time = loop_time;
  }
  
  // Log warning if loop takes too long (could cause packet loss) - DISABLED
  // if (loop_time > 50) {  // More than 50ms could cause packet buffer overflow
  //   unsigned long free_heap = ESP.getFreeHeap();
  //   unsigned long min_free_heap = ESP.getMinFreeHeap();
  //   Serial.printf("WARNING: Loop took %lums (mesh: %lums, timesync: %lums, max: %lums) - Risk of packet loss!\n", 
  //                 loop_time, mesh_time, timesync_time, max_loop_time);
  //   Serial.printf("HEAP: free=%lu, min_free=%lu, queue_size=%u\n", free_heap, min_free_heap, messageQueue.size());
  // }
  
  // Reset max every 10 seconds
  if (millis() - last_loop_time > 10000) {
    if (max_loop_time > 20) {
      MESH_DEBUG_PRINTLN("Loop timing: max %lums in last 10s", max_loop_time);
    }
    max_loop_time = 0;
    last_loop_time = millis();
  }
}
