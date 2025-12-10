#include "timesync_monitor.h"
#include <string.h>

// Forward declarations for enhanced mesh implementations
class LoggerMeshTables;  // Used in simple_web_logger
class MyMesh;            // Used in simple_web_logger

#ifdef TIMESYNC_MONITOR_ENABLED

#if defined(ESP32) || defined(RP2040_PLATFORM)
  #include <FS.h>
  #define FILESYSTEM  fs::FS
#elif defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <Adafruit_LittleFS.h>
  #define FILESYSTEM  Adafruit_LittleFS
  using namespace Adafruit_LittleFS_Namespace;
#endif

#define MAX_TRACKED_REPEATERS 350
#define SEVEN_DAYS_SECONDS 604800
#define DAILY_REPORT_HOUR 9          // UTC time hour (24h format)
#define DAILY_REPORT_MINUTE 0         // UTC time minute
#define MESSAGE_DELAY_MS 2000
#define MAX_MESSAGE_LENGTH 90
#define MAX_PACKET_CACHE 50
#define CACHE_EXPIRE_TIME_MS 60000
#define CACHE_HASH_SIZE 6
#define STALE_NODE_TIMEOUT_SECONDS 172800  // 48 hours in seconds (48 * 60 * 60)

struct RepeaterTimeState {
    uint8_t public_key[32];          // Full public key (primary identifier - persistent across name changes)
    char name[32];                   // Current/last known repeater name (for display only)
    bool had_good_time;              // Ever had good time sync
    bool on_shame_list;              // Currently on shame list
    uint8_t consecutive_bad_count;   // Counter for consecutive bad time adverts (resets on good time)
    uint32_t last_check_time;        // Last time we checked this repeater
    bool is_active;                  // Slot is in use
};

struct PacketCacheEntry {
    uint8_t hash[CACHE_HASH_SIZE];
    uint32_t timestamp;
};

static RepeaterTimeState repeater_states[MAX_TRACKED_REPEATERS];
static const mesh::GroupChannel* public_channel = nullptr;
static uint32_t last_report_day = 0;  // Track which day we last reported
static PacketCacheEntry packet_cache[MAX_PACKET_CACHE];
static uint8_t cache_index = 0;
static FILESYSTEM* fs_instance = nullptr;
static const char* fs_storage_path = "/timesync_good";
static bool save_pending = false;

// Simplified: Use UTC time directly, no timezone/DST calculations

static bool isPacketExpired(uint32_t timestamp) {
    uint32_t current_time = millis();
    if (current_time >= timestamp) {
        return (current_time - timestamp) >= CACHE_EXPIRE_TIME_MS;
    } else {
        uint32_t diff = (0xFFFFFFFF - timestamp) + current_time + 1;
        return diff >= CACHE_EXPIRE_TIME_MS;
    }
}

// Forward declaration removed - no longer needed

void TimeSyncMonitor::begin(void* filesystem, const char* storage_path) {
    memset(repeater_states, 0, sizeof(repeater_states));
    memset(packet_cache, 0, sizeof(packet_cache));
    last_report_day = 0;
    cache_index = 0;
    save_pending = false;
    
    if (filesystem) {
        fs_instance = (FILESYSTEM*)filesystem;
        if (storage_path) {
            fs_storage_path = storage_path;
        }
        loadGoodTimeList();
    }
}

void TimeSyncMonitor::processPendingSaves() {
    if (save_pending) {
        save_pending = false;
        saveGoodTimeList();
    }
}

void TimeSyncMonitor::processPendingSavesAsync() {
    // Non-blocking version: just mark that save is needed
    // The actual save will happen in a background task or during idle time
    if (save_pending) {
        // For now, we'll still do the save but could be moved to background task
        // This is a placeholder for future async implementation
        save_pending = false;
        saveGoodTimeList();
    }
}

void TimeSyncMonitor::loadGoodTimeList() {
    if (!fs_instance || !fs_instance->exists(fs_storage_path)) {
        return;
    }
    
    File file = fs_instance->open(fs_storage_path);
    if (!file) {
        return;
    }
    
    while (file.available()) {
        uint8_t public_key[32];
        char name[32];
        uint8_t had_good_time_byte;
        
        bool success = (file.read(public_key, 32) == 32);
        success = success && (file.read((uint8_t*)name, 32) == 32);
        success = success && (file.read(&had_good_time_byte, 1) == 1);
        
        if (!success) break;
        
        // Find empty slot
        for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
            if (!repeater_states[i].is_active) {
                repeater_states[i].is_active = true;
                memcpy(repeater_states[i].public_key, public_key, 32);
                strncpy(repeater_states[i].name, name, sizeof(repeater_states[i].name) - 1);
                repeater_states[i].name[sizeof(repeater_states[i].name) - 1] = '\0';
                repeater_states[i].had_good_time = (had_good_time_byte != 0);
                repeater_states[i].on_shame_list = false;  // Always start clean after reboot
                repeater_states[i].consecutive_bad_count = 0;  // Reset counter
                repeater_states[i].last_check_time = 0;
                break;
            }
        }
    }
    
    file.close();
}

void TimeSyncMonitor::saveGoodTimeList() {
    if (!fs_instance) {
        return;
    }
    
    File file = fs_instance->open(fs_storage_path, "w", true);
    if (!file) {
        return;
    }
    
    for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
        if (repeater_states[i].is_active && repeater_states[i].had_good_time) {
            uint8_t had_good_time_byte = repeater_states[i].had_good_time ? 1 : 0;
            
            bool success = (file.write(repeater_states[i].public_key, 32) == 32);
            success = success && (file.write((uint8_t*)repeater_states[i].name, 32) == 32);
            success = success && (file.write(&had_good_time_byte, 1) == 1);
            
            if (!success) break;
        }
    }
    
    file.close();
}

bool TimeSyncMonitor::hasRespondedToPacket(const uint8_t* packet_hash) {
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

void TimeSyncMonitor::markPacketResponded(const uint8_t* packet_hash) {
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

void TimeSyncMonitor::setPublicChannel(const mesh::GroupChannel* channel) {
    public_channel = channel;
}

bool TimeSyncMonitor::isNodeTimeSynced(uint32_t node_time) {
    // Node considers itself synced if its time is past the sanity check date
    return node_time >= TIMESYNC_SANITY_CHECK_EPOCH;
}

bool TimeSyncMonitor::isTimeGood(uint32_t node_time, uint32_t advertised_time) {
    // Time is good if:
    // 1. Year >= 2025 (timestamp >= 1735689600 = Jan 1, 2025 00:00:00 UTC)
    if (advertised_time < 1735689600) {
        return false;
    }
    
    // 2. Within 7 days of current node time (before or after)
    int64_t time_diff = (int64_t)advertised_time - (int64_t)node_time;
    if (time_diff < 0) time_diff = -time_diff;  // abs value
    
    if (time_diff > SEVEN_DAYS_SECONDS) {
        return false;
    }
    
    return true;
}

void TimeSyncMonitor::processAdvertisement(const mesh::Packet* packet, const mesh::Identity& id, 
                                          uint32_t advertised_timestamp, const char* node_name, uint32_t node_current_time, uint8_t adv_type) {
    if (!packet || !node_name || strlen(node_name) == 0) {
        return;
    }
    
    // Only track repeaters, ignore companions (ADV_TYPE_CHAT), room servers, sensors, etc.
    if (adv_type != ADV_TYPE_REPEATER) {
        return;
    }
    
    // Check if advertised time is good
    bool time_is_good = isTimeGood(node_current_time, advertised_timestamp);
    
    // Find or create slot for this repeater (search by public key)
    int slot_index = -1;
    int empty_slot = -1;
    
    for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
        if (repeater_states[i].is_active && memcmp(repeater_states[i].public_key, id.pub_key, 32) == 0) {
            // Found existing entry by public key
            slot_index = i;
            break;
        }
        if (!repeater_states[i].is_active && empty_slot == -1) {
            empty_slot = i;
        }
    }
    
    // If not found and we have space, create new entry
    if (slot_index == -1 && empty_slot != -1) {
        slot_index = empty_slot;
        repeater_states[slot_index].is_active = true;
        memcpy(repeater_states[slot_index].public_key, id.pub_key, 32);
        strncpy(repeater_states[slot_index].name, node_name, sizeof(repeater_states[slot_index].name) - 1);
        repeater_states[slot_index].name[sizeof(repeater_states[slot_index].name) - 1] = '\0';
        repeater_states[slot_index].had_good_time = false;
        repeater_states[slot_index].on_shame_list = false;
        repeater_states[slot_index].consecutive_bad_count = 0;
    }
    
    // If we couldn't find or create a slot, just return
    if (slot_index == -1) {
        return;
    }
    
    RepeaterTimeState* state = &repeater_states[slot_index];
    state->last_check_time = node_current_time;
    
    // Always update name to current advertised name (handles name changes)
    if (strcmp(state->name, node_name) != 0) {
        strncpy(state->name, node_name, sizeof(state->name) - 1);
        state->name[sizeof(state->name) - 1] = '\0';
    }
    
    // Logic:
    // - If time is good: mark had_good_time=true, remove from shame list, reset bad counter
    // - If time is bad AND had_good_time=true before: increment bad counter
    // - Add to shame list only after 2 consecutive bad time adverts
    
    bool state_changed = false;
    
    if (time_is_good) {
        if (!state->had_good_time) {
            state_changed = true;
        }
        state->had_good_time = true;
        state->on_shame_list = false;  // Redemption!
        state->consecutive_bad_count = 0;  // Reset bad counter
    } else {
        // Time is bad
        if (state->had_good_time) {
            // They had good time before, but now it's bad
            state->consecutive_bad_count++;
            
            // Only shame after 2 consecutive bad adverts (prevents false positives)
            // Defensive check: only shame if they actually had good time
            if (state->consecutive_bad_count >= 2 && state->had_good_time) {
                state->on_shame_list = true;
            }
        } else {
            // Never had good time - ensure they're not on shame list
            // This handles edge cases where state might be corrupted
            state->on_shame_list = false;
            state->consecutive_bad_count = 0;
        }
    }
    
    // Mark save needed when had_good_time changes (non-blocking)
    if (state_changed) {
        save_pending = true;
    }
}

void TimeSyncMonitor::sendShameListMessage(BaseChatMesh& mesh, const char* message, const char* node_name) {
    if (!public_channel || !node_name) {
        return;
    }
    
    // Use staggered delays to prevent simultaneous messages
    static uint8_t message_counter = 0;
    uint32_t delay = MESSAGE_DELAY_MS + (message_counter * 1000); // Stagger by 1 second each
    message_counter = (message_counter + 1) % 10; // Reset after 10 messages
    
    // Try to use enhanced channel message sending if available
    // This will be overridden in MyMesh to use retry system
    if (mesh.sendChannelMessage(*public_channel, message, node_name, delay)) {
        MESH_DEBUG_PRINTLN("[TIMESYNC-CH] Using enhanced channel message sending");
        return;
    }
    
    // Fallback to regular channel message sending
    uint8_t temp[5+MAX_TEXT_LEN+32];
    uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;  // TXT_TYPE_PLAIN

    // Format: <sender_name>: <response>
    sprintf((char *) &temp[5], "%s: %s", node_name, message);
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long

    int len = strlen((char *) &temp[5]);
    auto pkt = mesh.createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, *public_channel, temp, 5 + len);
    if (!pkt) {
        MESH_DEBUG_PRINTLN("[TIMESYNC-CH] ERROR: Failed to create group datagram (packet pool full?)");
        return;
    }
    
    MESH_DEBUG_PRINTLN("[TIMESYNC-CH] Using regular channel message sending");
    mesh.sendFlood(pkt, delay);
}

bool TimeSyncMonitor::processShameListCommand(BaseChatMesh& mesh, const ContactInfo& from,
                                              mesh::Packet* packet, uint32_t sender_timestamp, const char* text) {
    // Check if this is the shame list command (case insensitive)
    if (strcasecmp(text, "timesyncshamelist") != 0) {
        return false;
    }
    
    // Calculate packet hash for deduplication
    uint8_t packet_hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(packet_hash);
    
    // Check if we already responded to this packet
    if (hasRespondedToPacket(packet_hash)) {
        return false;
    }
    
    // Mark packet as responded (prevents duplicates)
    markPacketResponded(packet_hash);
    
    // Generate the shame list message (allow_empty=true for direct responses)
    uint32_t current_time = mesh.getRTCClock()->getCurrentTime();
    char shame_message[512];
    if (generateShameListMessage(shame_message, sizeof(shame_message), current_time, true)) {
        // Use the same limit as group messages for consistency
        int text_len = strlen(shame_message);
        if (text_len <= MAX_MESSAGE_LENGTH) {
            // Message fits in one packet, send directly
            uint8_t temp[5+MAX_TEXT_LEN+1];
            uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
            memcpy(temp, &timestamp, 4);
            temp[4] = 0;  // attempt = 0
            memcpy(&temp[5], shame_message, text_len + 1);
            
            mesh::Packet* response_pkt = mesh.createDatagram(PAYLOAD_TYPE_TXT_MSG, from.id, from.shared_secret, temp, 5 + text_len);
            
            if (response_pkt) {
                // Send with 2-second delay using MeshCore's built-in system
                if (from.out_path_len < 0) {
                    mesh.sendFlood(response_pkt, MESSAGE_DELAY_MS);
                } else {
                    mesh.sendDirect(response_pkt, from.out_path, from.out_path_len, MESSAGE_DELAY_MS);
                }
            }
        } else {
            // Message is too long, split it into multiple messages (same logic as group messages)
            char* line_start = shame_message;
            char current_msg[256];
            int current_pos = 0;
            bool is_first_message = true;
            
            // Skip the first line (header) since generateShameListMessage already includes it
            char* line_end = strchr(line_start, '\n');
            if (line_end != NULL) {
                line_start = line_end + 1;  // Skip header line
                
                // Start with the header for the first message only
                strncpy(current_msg, "SHAME 🔔 SYNC YOUR TIME!", sizeof(current_msg) - 1);
                current_pos = strlen(current_msg);
            }
            
            // Process each line (starting from second line - first name)
            line_end = strchr(line_start, '\n');
            while (line_end != NULL) {
                int line_len = line_end - line_start;
                
                // Check if adding this line would exceed the limit
                if (current_pos + 1 + line_len > MAX_MESSAGE_LENGTH) {
                    // Send current message and start new one
                    current_msg[current_pos] = '\0';
                    
                    uint8_t temp[5+MAX_TEXT_LEN+1];
                    uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
                    memcpy(temp, &timestamp, 4);
                    temp[4] = 0;  // attempt = 0
                    memcpy(&temp[5], current_msg, strlen(current_msg) + 1);
                    
                    mesh::Packet* response_pkt = mesh.createDatagram(PAYLOAD_TYPE_TXT_MSG, from.id, from.shared_secret, temp, 5 + strlen(current_msg));
                    
                    if (response_pkt) {
                        if (from.out_path_len < 0) {
                            mesh.sendFlood(response_pkt, MESSAGE_DELAY_MS);
                        } else {
                            mesh.sendDirect(response_pkt, from.out_path, from.out_path_len, MESSAGE_DELAY_MS);
                        }
                    }
                    
                    // Start new message WITHOUT header (just continuation of names)
                    current_msg[0] = '\0';
                    current_pos = 0;
                    is_first_message = false;
                }
                
                // Add newline and line content
                if (current_pos + 1 + line_len < (int)sizeof(current_msg) - 1) {
                    if (current_pos > 0) {
                        current_msg[current_pos++] = '\n';
                    }
                    memcpy(&current_msg[current_pos], line_start, line_len);
                    current_pos += line_len;
                }
                
                // Move to next line
                line_start = line_end + 1;
                line_end = strchr(line_start, '\n');
            }
            
            // Handle remaining content (last line without trailing newline)
            if (*line_start != '\0') {
                int remaining_len = strlen(line_start);
                if (remaining_len > 0) {
                    // Check if adding this would exceed the limit
                    if (current_pos + 1 + remaining_len > MAX_MESSAGE_LENGTH) {
                        // Send current message first
                        if (current_pos > 0) {
                            current_msg[current_pos] = '\0';
                            
                            uint8_t temp[5+MAX_TEXT_LEN+1];
                            uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
                            memcpy(temp, &timestamp, 4);
                            temp[4] = 0;  // attempt = 0
                            memcpy(&temp[5], current_msg, strlen(current_msg) + 1);
                            
                            mesh::Packet* response_pkt = mesh.createDatagram(PAYLOAD_TYPE_TXT_MSG, from.id, from.shared_secret, temp, 5 + strlen(current_msg));
                            
                            if (response_pkt) {
                                if (from.out_path_len < 0) {
                                    mesh.sendFlood(response_pkt, MESSAGE_DELAY_MS);
                                } else {
                                    mesh.sendDirect(response_pkt, from.out_path, from.out_path_len, MESSAGE_DELAY_MS);
                                }
                            }
                        }
                        
                        // Start new message for remaining content
                        current_msg[0] = '\0';
                        current_pos = 0;
                    }
                    
                    // Add remaining content
                    if (current_pos + 1 + remaining_len < (int)sizeof(current_msg) - 1) {
                        if (current_pos > 0) {
                            current_msg[current_pos++] = '\n';
                        }
                        memcpy(&current_msg[current_pos], line_start, remaining_len);
                        current_pos += remaining_len;
                    }
                }
            }
            
            // Send final message if there's content
            if (current_pos > 0) {
                current_msg[current_pos] = '\0';
                
                uint8_t temp[5+MAX_TEXT_LEN+1];
                uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
                memcpy(temp, &timestamp, 4);
                temp[4] = 0;  // attempt = 0
                memcpy(&temp[5], current_msg, strlen(current_msg) + 1);
                
                mesh::Packet* response_pkt = mesh.createDatagram(PAYLOAD_TYPE_TXT_MSG, from.id, from.shared_secret, temp, 5 + strlen(current_msg));
                
                if (response_pkt) {
                    if (from.out_path_len < 0) {
                        mesh.sendFlood(response_pkt, MESSAGE_DELAY_MS);
                    } else {
                        mesh.sendDirect(response_pkt, from.out_path, from.out_path_len, MESSAGE_DELAY_MS);
                    }
                }
            }
        }
    }
    
    return true;
}

bool TimeSyncMonitor::processClearCommand(BaseChatMesh& mesh, const ContactInfo& from,
                                         mesh::Packet* packet, uint32_t sender_timestamp, const char* text) {
    // Check if this is the clear command (case insensitive)
    if (strcasecmp(text, "cleartimesync") != 0) {
        return false;
    }
    
    // Calculate packet hash for deduplication
    uint8_t packet_hash[MAX_HASH_SIZE];
    packet->calculatePacketHash(packet_hash);
    
    // Check if we already responded to this packet
    if (hasRespondedToPacket(packet_hash)) {
        return false;
    }
    
    // Mark packet as responded (prevents duplicates)
    markPacketResponded(packet_hash);
    
    // Clear all timesync data
    clearAllData();
    
    // Send confirmation response
    char response[128];
    snprintf(response, sizeof(response), "Timesync data cleared (shame list and database reset)");
    
    int text_len = strlen(response);
    uint8_t temp[5+MAX_TEXT_LEN+1];
    uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;  // attempt = 0
    memcpy(&temp[5], response, text_len + 1);
    
    mesh::Packet* response_pkt = mesh.createDatagram(PAYLOAD_TYPE_TXT_MSG, from.id, from.shared_secret, temp, 5 + text_len);
    
    if (response_pkt) {
        // Send with 2-second delay using MeshCore's built-in system
        if (from.out_path_len < 0) {
            mesh.sendFlood(response_pkt, MESSAGE_DELAY_MS);
        } else {
            mesh.sendDirect(response_pkt, from.out_path, from.out_path_len, MESSAGE_DELAY_MS);
        }
    }
    
    return true;
}

void TimeSyncMonitor::clearAllData() {
    // Clear all in-memory state
    memset(repeater_states, 0, sizeof(repeater_states));
    memset(packet_cache, 0, sizeof(packet_cache));
    last_report_day = 0;
    cache_index = 0;
    save_pending = false;
    
    // Delete the persistent storage file if it exists
    if (fs_instance && fs_instance->exists(fs_storage_path)) {
        fs_instance->remove(fs_storage_path);
    }
}

bool TimeSyncMonitor::generateShameListMessage(char* output_buffer, size_t buffer_size, uint32_t current_time, bool allow_empty) {
    if (!output_buffer || buffer_size < 64) {
        return false;
    }
    
    // Build shame list with newlines
    int pos = snprintf(output_buffer, buffer_size, "SHAME 🔔 SYNC YOUR TIME!");
    
    bool has_offenders = false;
    for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
        if (repeater_states[i].is_active && repeater_states[i].on_shame_list) {
            // Check if node is stale (not seen in last 48 hours)
            // Use absolute value to handle time wrap-around
            uint32_t time_since_last_check;
            if (current_time >= repeater_states[i].last_check_time) {
                time_since_last_check = current_time - repeater_states[i].last_check_time;
            } else {
                // Handle time wrap-around (unlikely but possible)
                time_since_last_check = (0xFFFFFFFF - repeater_states[i].last_check_time) + current_time + 1;
            }
            
            // Skip stale nodes (older than 48 hours)
            if (time_since_last_check > STALE_NODE_TIMEOUT_SECONDS) {
                continue;
            }
            
            has_offenders = true;
            
            // Add newline before name
            if (pos < (int)buffer_size - 2) {
                output_buffer[pos++] = '\n';
            }
            
            // Add name
            int name_len = strlen(repeater_states[i].name);
            if (pos + name_len < (int)buffer_size - 1) {
                strcpy(&output_buffer[pos], repeater_states[i].name);
                pos += name_len;
            }
        }
    }
    
    if (!has_offenders) {
        if (allow_empty) {
            // Send empty message for direct responses
            snprintf(output_buffer, buffer_size, "SHAME 🔔 SYNC YOUR TIME!\n(empty - all repeaters in sync!)");
            return true;
        } else {
            // Don't send message if list is empty (for daily reports)
            return false;
        }
    }
    
    output_buffer[pos] = '\0';
    return true;
}

void TimeSyncMonitor::processDelayedResponses() {
    // This function is now empty since we use MeshCore's built-in delayed send system
    // Direct responses are handled immediately in processShameListCommand()
    // Channel messages are still processed in checkAndSendDailyReport()
}

void TimeSyncMonitor::checkAndSendDailyReport(BaseChatMesh& mesh, uint32_t current_time, const char* node_name) {
    if (!public_channel) {
        return;
    }
    
    // Only report if our own node is synced
    if (!isNodeTimeSynced(current_time)) {
        return;
    }
    
    // Use UTC time directly (no timezone calculations)
    uint32_t utc_time = current_time;
    
    // Calculate current day (days since epoch in UTC)
    uint32_t current_day = utc_time / 86400;
    
    // Early exit optimization: only proceed if we haven't reported today
    if (current_day == last_report_day) {
        return;  // Already reported today, skip time calculations
    }
    
    // Calculate current hour and minute in UTC
    uint32_t seconds_today = utc_time % 86400;
    uint32_t current_hour = seconds_today / 3600;
    uint32_t current_minute = (seconds_today % 3600) / 60;
    
    // Check if it's report time (9:00 AM UTC)
    if (current_hour == DAILY_REPORT_HOUR && current_minute == DAILY_REPORT_MINUTE) {
        last_report_day = current_day;
        
        // Build shame list using the same format as direct messages (allow_empty=false for daily reports)
        char shame_message[512];
        if (generateShameListMessage(shame_message, sizeof(shame_message), current_time, false)) {
            // Check if message is too long and needs to be split
            int text_len = strlen(shame_message);
            if (text_len <= MAX_MESSAGE_LENGTH) {
                // Message fits in one packet, send directly
                sendShameListMessage(mesh, shame_message, node_name);
            } else {
                // Message is too long, split it into multiple messages
                // Split by newlines to preserve the format
                char* line_start = shame_message;
                char current_msg[256];
                int current_pos = 0;
                bool is_first_message = true;
                
                // Skip the first line (header) since generateShameListMessage already includes it
                char* line_end = strchr(line_start, '\n');
                if (line_end != NULL) {
                    line_start = line_end + 1;  // Skip header line
                    
                    // Start with the header for the first message only
                    strncpy(current_msg, "SHAME 🔔 SYNC YOUR TIME!", sizeof(current_msg) - 1);
                    current_pos = strlen(current_msg);
                }
                
                // Process each line (starting from second line - first name)
                line_end = strchr(line_start, '\n');
                while (line_end != NULL) {
                    int line_len = line_end - line_start;
                    
                    // Check if adding this line would exceed the limit
                    if (current_pos + 1 + line_len > MAX_MESSAGE_LENGTH) {
                        // Send current message and start new one
                        current_msg[current_pos] = '\0';
                        sendShameListMessage(mesh, current_msg, node_name);
                        
                        // Start new message WITHOUT header (just the continuation of names)
                        current_msg[0] = '\0';
                        current_pos = 0;
                        is_first_message = false;
                    }
                    
                    // Add newline and line content
                    if (current_pos + 1 + line_len < (int)sizeof(current_msg) - 1) {
                        if (current_pos > 0) {
                            current_msg[current_pos++] = '\n';
                        }
                        memcpy(&current_msg[current_pos], line_start, line_len);
                        current_pos += line_len;
                    }
                    
                    // Move to next line
                    line_start = line_end + 1;
                    line_end = strchr(line_start, '\n');
                }
                
                // Handle remaining content (last line without trailing newline)
                if (*line_start != '\0') {
                    int remaining_len = strlen(line_start);
                    if (remaining_len > 0) {
                        // Check if adding this would exceed the limit
                        if (current_pos + 1 + remaining_len > MAX_MESSAGE_LENGTH) {
                            // Send current message first
                            if (current_pos > 0) {
                                current_msg[current_pos] = '\0';
                                sendShameListMessage(mesh, current_msg, node_name);
                            }
                            
                            // Start new message for remaining content
                            current_msg[0] = '\0';
                            current_pos = 0;
                        }
                        
                        // Add remaining content
                        if (current_pos + 1 + remaining_len < (int)sizeof(current_msg) - 1) {
                            if (current_pos > 0) {
                                current_msg[current_pos++] = '\n';
                            }
                            memcpy(&current_msg[current_pos], line_start, remaining_len);
                            current_pos += remaining_len;
                        }
                    }
                }
                
                // Send final message if there's content
                if (current_pos > 0) {
                    current_msg[current_pos] = '\0';
                    sendShameListMessage(mesh, current_msg, node_name);
                }
            }
        }
    }
}

#endif // TIMESYNC_MONITOR_ENABLED

