#include "pingpong.h"
#include <string.h>
#include <ctype.h>

#ifdef PINGPONG_ENABLED

bool PingPongHelper::isPingMessage(const char* text) {
    if (!text) return false;
    
    // Convert to lowercase for case-insensitive comparison
    size_t len = strlen(text);
    char* lower_text = new char[len + 1];
    for (size_t i = 0; i < len; i++) {
        lower_text[i] = tolower(text[i]);
    }
    lower_text[len] = '\0';
    
    // Look for " ping " (with spaces before and after)
    bool found = false;
    const char* ping_pos = strstr(lower_text, " ping ");
    if (ping_pos) {
        found = true;
    } else {
        // Also check if it starts with "ping " or ends with " ping"
        if (strncmp(lower_text, "ping ", 5) == 0) {
            found = true;
        } else if (len >= 5 && strcmp(&lower_text[len - 5], " ping") == 0) {
            found = true;
        }
    }
    
    delete[] lower_text;
    return found;
}

bool PingPongHelper::generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                        const char* router_ids, float snr, float rssi, char* response_buffer, size_t buffer_size) {
    if (!sender_name || !router_ids || !response_buffer || buffer_size < 64) {
        return false;
    }
    
    // Format: @[SENDER_NAME] Pong! X hops (router_ids) [SNR: X.X dB, RSSI: XX dBm]
    int written;
    if (hop_count == 0) {
        // Direct message - include SNR and RSSI
        written = snprintf(response_buffer, buffer_size, "@[%s] Pong! %d hops (%s) [SNR: %.1f dB, RSSI: %.0f dBm]", 
                          sender_name, hop_count, router_ids, snr, rssi);
    } else {
        // Multi-hop message - no SNR/RSSI for individual hops
        written = snprintf(response_buffer, buffer_size, "@[%s] Pong! %d hops (%s)", 
                          sender_name, hop_count, router_ids);
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
        // Direct message, no hops
        strncpy(router_ids_buffer, "direct", buffer_size - 1);
        router_ids_buffer[buffer_size - 1] = '\0';
        return true;
    }
    
    // Convert path bytes to hex strings
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
    Serial.printf("[PINGPONG] Received message: '%s'\n", text);
    
    if (!isPingMessage(text)) {
        Serial.printf("[PINGPONG] Not a ping message\n");
        return false;
    }
    
    Serial.printf("[PINGPONG] Detected ping message from %s\n", from.name);
    
    // Extract path information
    uint8_t hop_count;
    char router_ids[64];
    
    if (!extractPathInfo(packet, hop_count, router_ids, sizeof(router_ids))) {
        return false;
    }
    
    // Get SNR and RSSI from packet and radio
    float snr = packet->getSNR();
    float rssi = mesh.getRadio()->getLastRSSI();
    
    // Generate pong response
    char response[128];
    if (!generatePongResponse(from.name, hop_count, router_ids, snr, rssi, response, sizeof(response))) {
        Serial.printf("[PINGPONG] Failed to generate pong response\n");
        return false;
    }
    
    Serial.printf("[PINGPONG] Generated response: '%s'\n", response);
    
    // Send the pong response
    uint32_t est_timeout;
    uint32_t expected_ack_crc;
    
    int result = mesh.sendMessage(from, mesh.getRTCClock()->getCurrentTime(), 0, 
                                response, expected_ack_crc, est_timeout);
    
    if (result != MSG_SEND_FAILED) {
        Serial.printf("[PINGPONG] Successfully sent pong response\n");
    } else {
        Serial.printf("[PINGPONG] Failed to send pong response\n");
    }
    
    return result != MSG_SEND_FAILED;
}

#endif // PINGPONG_ENABLED
