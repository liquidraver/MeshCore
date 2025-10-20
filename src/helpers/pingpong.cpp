#include "pingpong.h"
#include <string.h>
#include <ctype.h>

#ifdef PINGPONG_ENABLED

#define MAX_SENDER_COOLDOWN 10
#define SENDER_COOLDOWN_EXPIRE_MS 60000

struct SenderCooldown {
    char sender_name[32];
    uint32_t last_response_time;
    bool is_active;
};

static SenderCooldown sender_cooldowns[MAX_SENDER_COOLDOWN];

void PingPongHelper::begin() {
    memset(sender_cooldowns, 0, sizeof(sender_cooldowns));
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

    Serial.printf("[PING] Detected ping from %s\n", from.name);
    
    // Per-sender 15-second cooldown (prevents spam responses)
    bool canRespond = canRespondToChannelSender(from.name, 15000);
    if (!canRespond) {
        Serial.printf("[PING] Skipping response to %s (still in cooldown)\n", from.name);
        return false;
    }
    
    uint8_t hop_count = packet->path_len;
    char router_ids_buffer[256];
    
    if (!extractPathInfo(packet, hop_count, router_ids_buffer, sizeof(router_ids_buffer))) {
        Serial.println("[PING] ERROR: Failed to extract path info");
        return false;
    }
    
    float rssi = 0.0;
    if (mesh.getRadio()) {
        rssi = mesh.getRadio()->getLastRSSI();
    }

    char response[256];
    if (!generatePongResponse(from.name, hop_count, router_ids_buffer, packet->getSNR(), rssi, false, response, sizeof(response))) {
        Serial.println("[PING] ERROR: Failed to generate pong response");
        return false;
    }
    
    Serial.printf("[PING] Generated response: %s\n", response);
    
    // Use the existing sendMessage() method which handles ACK timeout and retry automatically
    uint32_t est_timeout;
    uint32_t expected_ack;
    
    // Randomized delay 8-10 seconds to prevent simultaneous responses
    uint32_t delay = mesh.getRNG()->nextInt(8000, 10001);
    
    Serial.printf("[PING] Sending pong to %s with %dms delay (path_len=%d)\n", 
                  from.name, delay, from.out_path_len);
    
    // Use sendMessage with attempt=0 (it will retry automatically on ACK timeout)
    int result = mesh.sendMessage(from, mesh.getRTCClock()->getCurrentTime(), 0, response, expected_ack, est_timeout);
    
    if (result == MSG_SEND_FAILED) {
        Serial.println("[PING] ERROR: Failed to send pong message");
        return false;
    } else {
        Serial.printf("[PING] Pong queued - %s (timeout: %ums)\n", 
                      result == MSG_SEND_SENT_FLOOD ? "FLOOD" : "DIRECT", est_timeout);
    }
    
    return true;
}

void PingPongHelper::scheduleDelayedChannelResponse(BaseChatMesh& mesh, const mesh::GroupChannel& channel, 
                                                   const char* response, uint32_t delay_ms, const char* sender_name) {
    Serial.printf("[PING-CH] Scheduling channel pong for sender: %s\n", sender_name);
    
    // Create channel message packet and send with delay using MeshCore's built-in system
    uint8_t temp[5+MAX_TEXT_LEN+32];
    uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;  // TXT_TYPE_PLAIN

    // Format: <sender_name>: <response>
    sprintf((char *) &temp[5], "%s: %s", sender_name, response);
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

    int len = strlen((char *) &temp[5]);
    auto pkt = mesh.createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, channel, temp, 5 + len);
    if (!pkt) {
        Serial.println("[PING-CH] ERROR: Failed to create group datagram (packet pool full?)");
        return;
    }
    
    Serial.printf("[PING-CH] Queueing channel pong with %dms delay\n", delay_ms);
    // Use MeshCore's built-in delayed send
    mesh.sendFlood(pkt, delay_ms);
    Serial.println("[PING-CH] Queued successfully");
}

bool PingPongHelper::canRespondToChannelSender(const char* sender_name, uint32_t cooldown_ms) {
    if (!sender_name) return false;
    
    uint32_t current_time = millis();
    
    // Check if this sender is in cooldown
    for (int i = 0; i < MAX_SENDER_COOLDOWN; i++) {
        if (sender_cooldowns[i].is_active && strcmp(sender_cooldowns[i].sender_name, sender_name) == 0) {
            // Found sender, check if cooldown expired
            bool expired = false;
            if (current_time >= sender_cooldowns[i].last_response_time) {
                expired = (current_time - sender_cooldowns[i].last_response_time) >= cooldown_ms;
            } else {
                // Handle millis rollover
                uint32_t diff = (0xFFFFFFFF - sender_cooldowns[i].last_response_time) + current_time + 1;
                expired = diff >= cooldown_ms;
            }
            
            if (expired) {
                // Update timestamp and allow
                sender_cooldowns[i].last_response_time = current_time;
                return true;
            }
            return false; // Still in cooldown
        }
    }
    
    // Sender not found, add to cooldown list
    for (int i = 0; i < MAX_SENDER_COOLDOWN; i++) {
        // Find inactive or expired slot
        bool slot_available = !sender_cooldowns[i].is_active;
        if (!slot_available && current_time >= sender_cooldowns[i].last_response_time) {
            slot_available = (current_time - sender_cooldowns[i].last_response_time) >= SENDER_COOLDOWN_EXPIRE_MS;
        }
        
        if (slot_available) {
            sender_cooldowns[i].is_active = true;
            strncpy(sender_cooldowns[i].sender_name, sender_name, sizeof(sender_cooldowns[i].sender_name) - 1);
            sender_cooldowns[i].sender_name[sizeof(sender_cooldowns[i].sender_name) - 1] = '\0';
            sender_cooldowns[i].last_response_time = current_time;
            return true;
        }
    }
    
    // No slots available, use oldest (index 0)
    sender_cooldowns[0].is_active = true;
    strncpy(sender_cooldowns[0].sender_name, sender_name, sizeof(sender_cooldowns[0].sender_name) - 1);
    sender_cooldowns[0].sender_name[sizeof(sender_cooldowns[0].sender_name) - 1] = '\0';
    sender_cooldowns[0].last_response_time = current_time;
    return true;
}


#endif // PINGPONG_ENABLED
