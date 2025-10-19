#pragma once

#include <Mesh.h>
#include <helpers/SimpleMeshTables.h>

class LoggerMeshTables : public SimpleMeshTables {

private:
  // Track packets we've sent for retry detection
  struct SentPacket {
    uint8_t hash[MAX_HASH_SIZE];
    unsigned long sent_time;
    bool heard_repetition;
    uint8_t retry_count;
  };
  
  static const int MAX_SENT_PACKETS = 16;
  SentPacket sent_packets[MAX_SENT_PACKETS];
  int next_sent_idx;

public:
  LoggerMeshTables() : next_sent_idx(0) {
    memset(sent_packets, 0, sizeof(sent_packets));
  }

  bool hasSeen(const mesh::Packet* packet) override {
    // Check if this is a packet we sent that's coming back (repetition detection)
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    
    for (int i = 0; i < MAX_SENT_PACKETS; i++) {
      if (memcmp(hash, sent_packets[i].hash, MAX_HASH_SIZE) == 0) {
        // This is a packet we sent - mark it as heard
        sent_packets[i].heard_repetition = true;
        MESH_DEBUG_PRINTLN("[RETRY] Heard repetition of our packet (sent %lums ago)", 
                           millis() - sent_packets[i].sent_time);
        break;
      }
    }
    
    // Always false. This allows onRecvPacket to be called multiple times for
    // packet received from multiple routes.
    return false;
  }

  bool hasSeen2(const mesh::Packet* packet) {
    // This is used for forward check. We don't want to forward duplicates.
    return SimpleMeshTables::hasSeen(packet);
  }
  
  // Track a packet we're about to send
  void trackSentPacket(const mesh::Packet* packet) {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    
    memcpy(sent_packets[next_sent_idx].hash, hash, MAX_HASH_SIZE);
    sent_packets[next_sent_idx].sent_time = millis();
    sent_packets[next_sent_idx].heard_repetition = false;
    sent_packets[next_sent_idx].retry_count = 0;
    
    next_sent_idx = (next_sent_idx + 1) % MAX_SENT_PACKETS;
  }
  
  // Check if a packet we sent was heard (repeated by other nodes)
  bool wasPacketHeard(const mesh::Packet* packet) {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    
    for (int i = 0; i < MAX_SENT_PACKETS; i++) {
      if (memcmp(hash, sent_packets[i].hash, MAX_HASH_SIZE) == 0) {
        return sent_packets[i].heard_repetition;
      }
    }
    return false;
  }
  
  // Increment retry count for a packet
  void incrementRetryCount(const mesh::Packet* packet) {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    
    for (int i = 0; i < MAX_SENT_PACKETS; i++) {
      if (memcmp(hash, sent_packets[i].hash, MAX_HASH_SIZE) == 0) {
        sent_packets[i].retry_count++;
        break;
      }
    }
  }
  
  // Get retry count for a packet
  uint8_t getRetryCount(const mesh::Packet* packet) {
    uint8_t hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(hash);
    
    for (int i = 0; i < MAX_SENT_PACKETS; i++) {
      if (memcmp(hash, sent_packets[i].hash, MAX_HASH_SIZE) == 0) {
        return sent_packets[i].retry_count;
      }
    }
    return 0;
  }
};
