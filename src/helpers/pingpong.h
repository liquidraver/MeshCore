#pragma once

#include <Mesh.h>
#include <helpers/BaseChatMesh.h>

#ifdef PINGPONG_ENABLED

class PingPongHelper {
public:
    static void begin();
    static bool isPingMessage(const char* text);
    static bool generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                   const char* router_ids, float snr, float rssi, char* response_buffer, size_t buffer_size);
    static bool generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                   const char* router_ids, float snr, float rssi, bool is_channel_message,
                                   char* response_buffer, size_t buffer_size);
    static bool extractPathInfo(const mesh::Packet* packet, uint8_t& hop_count, 
                               char* router_ids_buffer, size_t buffer_size);
    static bool processMessage(BaseChatMesh& mesh, const ContactInfo& from,
                             mesh::Packet* packet, uint32_t sender_timestamp, const char* text);
    static void scheduleDelayedChannelResponse(BaseChatMesh& mesh, const mesh::GroupChannel& channel, 
                                             const char* response, uint32_t delay_ms, const char* sender_name);
    static bool canRespondToChannelSender(const char* sender_name, uint32_t cooldown_ms);
};

#endif // PINGPONG_ENABLED
