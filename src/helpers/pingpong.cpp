#include "pingpong.h"
#include <string.h>
#include <ctype.h>

#ifdef PINGPONG_ENABLED

#define MAX_PACKET_CACHE 50
#define CACHE_EXPIRE_TIME_MS 60000
#define CACHE_HASH_SIZE 6

struct PacketCacheEntry {
    uint8_t hash[CACHE_HASH_SIZE];
    uint32_t timestamp;
};

struct DelayedResponse {
    BaseChatMesh* mesh;
    ContactInfo contact;
    char response[256];
    uint32_t send_time;
    bool is_active;
};

struct DelayedChannelResponse {
    BaseChatMesh* mesh;
    mesh::GroupChannel channel;
    char response[256];
    char sender_name[32];
    uint32_t send_time;
    bool is_active;
};

static PacketCacheEntry packet_cache[MAX_PACKET_CACHE];
static uint8_t cache_index = 0;

static DelayedResponse delayed_responses[5];
static DelayedChannelResponse delayed_channel_responses[5];
static uint8_t response_count = 0;
static uint8_t channel_response_count = 0;

static bool isPacketExpired(uint32_t timestamp) {
    uint32_t current_time = millis();
    if (current_time >= timestamp) {
        return (current_time - timestamp) >= CACHE_EXPIRE_TIME_MS;
    } else {
        uint32_t diff = (0xFFFFFFFF - timestamp) + current_time + 1;
        return diff >= CACHE_EXPIRE_TIME_MS;
    }
}

bool PingPongHelper::hasRespondedToPacket(const uint8_t* packet_hash) {
    if (!packet_hash) return false;
    
    for (int i = 0; i < MAX_PACKET_CACHE; i++) {
        if (packet_cache[i].timestamp != 0 && 
            !isPacketExpired(packet_cache[i].timestamp)) {
            if (memcmp(packet_cache[i].hash, packet_hash, CACHE_HASH_SIZE) == 0) {
                return true;
            }
        }
    }
    return false;
}

void PingPongHelper::markPacketResponded(const uint8_t* packet_hash) {
    if (!packet_hash) return;
    
    int slot_to_use = cache_index;
    for (int i = 0; i < MAX_PACKET_CACHE; i++) {
        int check_index = (cache_index + i) % MAX_PACKET_CACHE;
        if (packet_cache[check_index].timestamp == 0 || 
            isPacketExpired(packet_cache[check_index].timestamp)) {
            slot_to_use = check_index;
            break;
        }
    }
    
    memcpy(packet_cache[slot_to_use].hash, packet_hash, CACHE_HASH_SIZE);
    packet_cache[slot_to_use].timestamp = millis();
    
    cache_index = (cache_index + 1) % MAX_PACKET_CACHE;
}

bool PingPongHelper::isPingMessage(const char* text) {
    if (!text) return false;
    
    size_t len = strlen(text);
    char* lower_text = new char[len + 1];
    for (size_t i = 0; i < len; i++) {
        lower_text[i] = tolower(text[i]);
    }
    lower_text[len] = '\0';
    
    if (strcmp(lower_text, "ping") == 0) {
        delete[] lower_text;
        return true;
    }
    
    const char* colon_pos = strchr(lower_text, ':');
    if (colon_pos) {
        const char* after_colon = colon_pos + 1;
        while (*after_colon == ' ') after_colon++;
        
        if (strcmp(after_colon, "ping") == 0) {
            delete[] lower_text;
            return true;
        }
    }
    
    delete[] lower_text;
    return false;
}

bool PingPongHelper::generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                        const char* router_ids, float snr, float rssi, char* response_buffer, size_t buffer_size) {
    if (!sender_name || !router_ids || !response_buffer || buffer_size < 64) {
        return false;
    }
    
    int written;
    if (hop_count == 0) {
        written = snprintf(response_buffer, buffer_size, "@[%s] Pong! %d hops (%s) [SNR: %.1f dB, RSSI: %.0f dBm]", 
                          sender_name, hop_count, router_ids, snr, rssi);
    } else {
        written = snprintf(response_buffer, buffer_size, "@[%s] Pong! %d hops (%s)", 
                          sender_name, hop_count, router_ids);
    }
    
    return written > 0 && written < (int)buffer_size;
}

bool PingPongHelper::generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                        const char* router_ids, float snr, float rssi, bool is_channel_message,
                                        char* response_buffer, size_t buffer_size) {
    if (!sender_name || !router_ids || !response_buffer || buffer_size < 64) {
        return false;
    }
    
    int written;
    if (hop_count == 0) {
        if (is_channel_message) {
            written = snprintf(response_buffer, buffer_size, "@[%s] Pong! 0 hops (direct) [SNR: %.1f dB, RSSI: %.0f dBm]", 
                              sender_name, snr, rssi);
        } else {
            written = snprintf(response_buffer, buffer_size, "Pong! 0 hops (direct) [SNR: %.1f dB, RSSI: %.0f dBm]", 
                              snr, rssi);
        }
    } else {
        if (is_channel_message) {
            written = snprintf(response_buffer, buffer_size, "@[%s] Pong! %d hops (%s)", 
                              sender_name, hop_count, router_ids);
        } else {
            written = snprintf(response_buffer, buffer_size, "Pong! %d hops (%s)", 
                              hop_count, router_ids);
        }
    }
    
    return written > 0 && written < (int)buffer_size;
}

bool PingPongHelper::extractPathInfo(const mesh::Packet* packet, uint8_t& hop_count, 
                                    char* router_ids_buffer, size_t buffer_size) {
    if (!packet || !router_ids_buffer || buffer_size < 16) {
        return false;
    }
    
    hop_count = packet->path_len;
    
    if (packet->path_len == 0) {
        strncpy(router_ids_buffer, "direct", buffer_size - 1);
        router_ids_buffer[buffer_size - 1] = '\0';
        return true;
    }
    
    size_t pos = 0;
    for (uint8_t i = 0; i < packet->path_len && pos < buffer_size - 3; i++) {
        if (i > 0) {
            router_ids_buffer[pos++] = ',';
        }
        int written = snprintf(&router_ids_buffer[pos], buffer_size - pos, "%02x", packet->path[i]);
        if (written > 0) {
            pos += written;
        }
    }
    router_ids_buffer[pos] = '\0';
    
    return true;
}

bool PingPongHelper::processMessage(BaseChatMesh& mesh, const ContactInfo& from,
                                   mesh::Packet* packet, uint32_t sender_timestamp, const char* text) {
    if (!isPingMessage(text)) {
        return false;
    }

    uint8_t packet_hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(packet_hash);
    
    if (hasRespondedToPacket(packet_hash)) {
        return false;
    }

    uint8_t hop_count = packet->path_len;
    char router_ids_buffer[256];
    
    if (!extractPathInfo(packet, hop_count, router_ids_buffer, sizeof(router_ids_buffer))) {
        return false;
    }
    
    float rssi = 0.0;
    if (mesh.getRadio()) {
        rssi = mesh.getRadio()->getLastRSSI();
    }

    char response[256];
    if (generatePongResponse(from.name, hop_count, router_ids_buffer, packet->getSNR(), rssi, false, response, sizeof(response))) {
        markPacketResponded(packet_hash);
        scheduleDelayedResponse(mesh, from, response, 5000);
        return true;
    }
    
    return false;
}

void PingPongHelper::scheduleDelayedResponse(BaseChatMesh& mesh, const ContactInfo& from, 
                                           const char* response, uint32_t delay_ms) {
    if (!response || response_count >= 5) {
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        if (!delayed_responses[i].is_active) {
            delayed_responses[i].mesh = &mesh;
            delayed_responses[i].contact = from;
            strncpy(delayed_responses[i].response, response, sizeof(delayed_responses[i].response) - 1);
            delayed_responses[i].response[sizeof(delayed_responses[i].response) - 1] = '\0';
            delayed_responses[i].send_time = millis() + delay_ms;
            delayed_responses[i].is_active = true;
            response_count++;
            break;
        }
    }
}

void PingPongHelper::scheduleDelayedChannelResponse(BaseChatMesh& mesh, const mesh::GroupChannel& channel, 
                                                   const char* response, uint32_t delay_ms, const char* sender_name) {
    if (!response || channel_response_count >= 5) {
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        if (!delayed_channel_responses[i].is_active) {
            delayed_channel_responses[i].mesh = &mesh;
            delayed_channel_responses[i].channel = channel;
            strncpy(delayed_channel_responses[i].response, response, sizeof(delayed_channel_responses[i].response) - 1);
            delayed_channel_responses[i].response[sizeof(delayed_channel_responses[i].response) - 1] = '\0';
            strncpy(delayed_channel_responses[i].sender_name, sender_name, sizeof(delayed_channel_responses[i].sender_name) - 1);
            delayed_channel_responses[i].sender_name[sizeof(delayed_channel_responses[i].sender_name) - 1] = '\0';
            delayed_channel_responses[i].send_time = millis() + delay_ms;
            delayed_channel_responses[i].is_active = true;
            channel_response_count++;
            break;
        }
    }
}

void PingPongHelper::processScheduledResponses() {
    for (int i = 0; i < 5; i++) {
        if (delayed_responses[i].is_active && millis() >= delayed_responses[i].send_time) {
            uint32_t expected_ack, est_timeout;
            int result = delayed_responses[i].mesh->sendMessage(
                delayed_responses[i].contact, delayed_responses[i].mesh->getRTCClock()->getCurrentTime(), 0, 
                delayed_responses[i].response, expected_ack, est_timeout);
            
            delayed_responses[i].is_active = false;
            response_count--;
        }
    }
    
    for (int i = 0; i < 5; i++) {
        if (delayed_channel_responses[i].is_active && millis() >= delayed_channel_responses[i].send_time) {
            delayed_channel_responses[i].mesh->sendGroupMessage(
                delayed_channel_responses[i].mesh->getRTCClock()->getCurrentTime(), const_cast<mesh::GroupChannel&>(delayed_channel_responses[i].channel), 
                delayed_channel_responses[i].sender_name, delayed_channel_responses[i].response, strlen(delayed_channel_responses[i].response));
            
            delayed_channel_responses[i].is_active = false;
            channel_response_count--;
        }
    }
}

#endif // PINGPONG_ENABLED
