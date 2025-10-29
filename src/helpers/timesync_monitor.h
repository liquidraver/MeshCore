#pragma once

#include <Mesh.h>
#include <helpers/BaseChatMesh.h>

// Sanity check: Consider time "good" if >= August 9, 2025
#define TIMESYNC_SANITY_CHECK_EPOCH 1754697600  // 2025-08-09 00:00:00 UTC

#ifdef TIMESYNC_MONITOR_ENABLED

class TimeSyncMonitor {
public:
    static void begin(void* filesystem = nullptr, const char* storage_path = nullptr);
    static void processAdvertisement(const mesh::Packet* packet, const mesh::Identity& id, 
                                    uint32_t advertised_timestamp, const char* node_name, uint32_t node_current_time);
    static void checkAndSendDailyReport(BaseChatMesh& mesh, uint32_t current_time, const char* node_name);
    static void processDelayedResponses();  // New function to process delayed responses separately
    static void processPendingSaves();
    static void processPendingSavesAsync();  // Non-blocking version
    static void setPublicChannel(const mesh::GroupChannel* channel);
    static bool generateShameListMessage(char* output_buffer, size_t buffer_size, uint32_t current_time, bool allow_empty = false);
    static bool processShameListCommand(BaseChatMesh& mesh, const ContactInfo& from,
                                       mesh::Packet* packet, uint32_t sender_timestamp, const char* text);
    static void saveGoodTimeList();
    static void loadGoodTimeList();
    
private:
    static bool isNodeTimeSynced(uint32_t node_time);
    static bool isTimeGood(uint32_t node_time, uint32_t advertised_time);
    static void sendShameListMessage(BaseChatMesh& mesh, const char* message, const char* node_name);
    static bool hasRespondedToPacket(const uint8_t* packet_hash);
    static void markPacketResponded(const uint8_t* packet_hash);
};

#endif // TIMESYNC_MONITOR_ENABLED

