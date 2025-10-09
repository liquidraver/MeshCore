#include "timesync_monitor.h"
#include <string.h>

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
#define DAILY_REPORT_HOUR 10          // Local time hour (24h format)
#define DAILY_REPORT_MINUTE 0         // Local time minute
#define MESSAGE_DELAY_MS 5000
#define MAX_MESSAGE_LENGTH 150
#define MAX_PACKET_CACHE 50
#define CACHE_EXPIRE_TIME_MS 60000
#define CACHE_HASH_SIZE 6

struct RepeaterTimeState {
    uint8_t public_key[32];          // Full public key (primary identifier - persistent across name changes)
    char name[32];                   // Current/last known repeater name (for display only)
    bool had_good_time;              // Ever had good time sync
    bool on_shame_list;              // Currently on shame list
    uint32_t last_check_time;        // Last time we checked this repeater
    bool is_active;                  // Slot is in use
};

struct DelayedShameMessage {
    BaseChatMesh* mesh;
    char message[256];
    uint32_t send_time;
    bool is_active;
};

struct DelayedDirectResponse {
    BaseChatMesh* mesh;
    ContactInfo contact;
    char response[512];
    uint32_t send_time;
    bool is_active;
};

struct PacketCacheEntry {
    uint8_t hash[CACHE_HASH_SIZE];
    uint32_t timestamp;
};

static RepeaterTimeState repeater_states[MAX_TRACKED_REPEATERS];
static const mesh::GroupChannel* public_channel = nullptr;
static uint32_t last_report_day = 0;  // Track which day we last reported
static DelayedShameMessage delayed_messages[10];  // Support up to 10 continuation messages
static uint8_t delayed_message_count = 0;
static DelayedDirectResponse delayed_direct_responses[5];  // Direct message responses
static uint8_t direct_response_count = 0;
static PacketCacheEntry packet_cache[MAX_PACKET_CACHE];
static uint8_t cache_index = 0;
static FILESYSTEM* fs_instance = nullptr;
static const char* fs_storage_path = "/timesync_good";

// Calculate if DST is active for Europe/Budapest at given UTC timestamp
// DST: Last Sunday of March 01:00 UTC -> Last Sunday of October 01:00 UTC
static bool isDSTActive(uint32_t utc_time) {
    // Calculate year and day of year
    uint32_t days_since_epoch = utc_time / 86400;
    uint32_t seconds_today = utc_time % 86400;
    
    // Approximate year (won't be exact but close enough for DST calculation)
    uint32_t year = 1970 + (days_since_epoch / 365);
    
    // Calculate Jan 1 of this year
    uint32_t years_since_1970 = year - 1970;
    uint32_t leap_years = (years_since_1970 + 1) / 4;  // Approximate leap years
    uint32_t jan1_days = (years_since_1970 * 365) + leap_years;
    
    // Days into current year
    int32_t day_of_year = days_since_epoch - jan1_days;
    
    // Handle year boundary issues
    if (day_of_year < 0) {
        year--;
        years_since_1970 = year - 1970;
        leap_years = (years_since_1970 + 1) / 4;
        jan1_days = (years_since_1970 * 365) + leap_years;
        day_of_year = days_since_epoch - jan1_days;
    }
    
    // Check if leap year
    bool is_leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    
    // Calculate day of week for Jan 1 (0 = Sunday)
    uint32_t jan1_dow = (jan1_days + 4) % 7;  // Jan 1, 1970 was Thursday (4)
    
    // Find last Sunday of March (day 59 or 60 in non-leap/leap year is March 1)
    uint32_t march1_day = is_leap ? 60 : 59;
    uint32_t march1_dow = (jan1_dow + march1_day) % 7;
    
    // Days in March = 31, so last day is March 31 = march1_day + 30
    uint32_t march31_day = march1_day + 30;
    uint32_t march31_dow = (march1_dow + 30) % 7;
    
    // Last Sunday of March
    uint32_t last_sunday_march = march31_day;
    if (march31_dow != 0) {  // If March 31 is not Sunday
        last_sunday_march -= march31_dow;
    }
    
    // Find last Sunday of October (day 273 or 274 in non-leap/leap year is Oct 1)
    uint32_t oct1_day = is_leap ? 274 : 273;
    uint32_t oct1_dow = (jan1_dow + oct1_day) % 7;
    
    // Days in October = 31, so last day is Oct 31 = oct1_day + 30
    uint32_t oct31_day = oct1_day + 30;
    uint32_t oct31_dow = (oct1_dow + 30) % 7;
    
    // Last Sunday of October
    uint32_t last_sunday_oct = oct31_day;
    if (oct31_dow != 0) {  // If Oct 31 is not Sunday
        last_sunday_oct -= oct31_dow;
    }
    
    // DST starts at 01:00 UTC on last Sunday of March
    // DST ends at 01:00 UTC on last Sunday of October
    bool after_dst_start = (day_of_year > last_sunday_march) || 
                           (day_of_year == last_sunday_march && seconds_today >= 3600);
    bool before_dst_end = (day_of_year < last_sunday_oct) || 
                          (day_of_year == last_sunday_oct && seconds_today < 3600);
    
    return after_dst_start && before_dst_end;
}

// Get Europe/Budapest timezone offset (automatically handles DST)
// Returns 1 (UTC+1) if node time is not synced yet
static int getBudapestTimezoneOffset(uint32_t utc_time) {
    // Only calculate DST if node has good time (after sanity check date)
    if (utc_time < TIMESYNC_SANITY_CHECK_EPOCH) {
        return 1;  // Default to UTC+1 if time not synced
    }
    return isDSTActive(utc_time) ? 2 : 1;  // UTC+2 in summer, UTC+1 in winter
}

static bool isPacketExpired(uint32_t timestamp) {
    uint32_t current_time = millis();
    if (current_time >= timestamp) {
        return (current_time - timestamp) >= CACHE_EXPIRE_TIME_MS;
    } else {
        uint32_t diff = (0xFFFFFFFF - timestamp) + current_time + 1;
        return diff >= CACHE_EXPIRE_TIME_MS;
    }
}

// Forward declaration
static void scheduleDelayedDirectResponse(BaseChatMesh& mesh, const ContactInfo& from, 
                                         const char* response, uint32_t delay_ms);

void TimeSyncMonitor::begin(void* filesystem, const char* storage_path) {
    memset(repeater_states, 0, sizeof(repeater_states));
    memset(delayed_messages, 0, sizeof(delayed_messages));
    memset(delayed_direct_responses, 0, sizeof(delayed_direct_responses));
    memset(packet_cache, 0, sizeof(packet_cache));
    delayed_message_count = 0;
    direct_response_count = 0;
    last_report_day = 0;
    cache_index = 0;
    
    if (filesystem) {
        fs_instance = (FILESYSTEM*)filesystem;
        if (storage_path) {
            fs_storage_path = storage_path;
        }
        loadGoodTimeList();
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
                                          uint32_t advertised_timestamp, const char* node_name, uint32_t node_current_time) {
    if (!packet || !node_name || strlen(node_name) == 0) {
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
    // - If time is good: mark had_good_time=true, remove from shame list
    // - If time is bad AND had_good_time=true before: add to shame list
    
    bool state_changed = false;
    
    if (time_is_good) {
        if (!state->had_good_time) {
            state_changed = true;
        }
        state->had_good_time = true;
        state->on_shame_list = false;  // Redemption!
    } else {
        // Time is bad
        if (state->had_good_time) {
            // They had good time before, but now it's bad - shame!
            state->on_shame_list = true;
        }
        // else: never had good time, don't add to shame list yet
    }
    
    // Save to filesystem when had_good_time changes
    if (state_changed) {
        saveGoodTimeList();
    }
}

void TimeSyncMonitor::sendShameListMessage(BaseChatMesh& mesh, const char* message) {
    if (!public_channel || delayed_message_count >= 10) {
        return;
    }
    
    // Find an empty slot
    for (int i = 0; i < 10; i++) {
        if (!delayed_messages[i].is_active) {
            delayed_messages[i].mesh = &mesh;
            strncpy(delayed_messages[i].message, message, sizeof(delayed_messages[i].message) - 1);
            delayed_messages[i].message[sizeof(delayed_messages[i].message) - 1] = '\0';
            delayed_messages[i].send_time = millis() + (delayed_message_count * MESSAGE_DELAY_MS);
            delayed_messages[i].is_active = true;
            delayed_message_count++;
            break;
        }
    }
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
    
    // Generate the shame list message
    char shame_message[512];
    if (generateShameListMessage(shame_message, sizeof(shame_message))) {
        // Schedule delayed response (allows ACK to be sent first)
        scheduleDelayedDirectResponse(mesh, from, shame_message, 5000);
    }
    
    return true;
}

static void scheduleDelayedDirectResponse(BaseChatMesh& mesh, const ContactInfo& from, 
                                         const char* response, uint32_t delay_ms) {
    if (!response || direct_response_count >= 5) {
        return;
    }
    
    for (int i = 0; i < 5; i++) {
        if (!delayed_direct_responses[i].is_active) {
            delayed_direct_responses[i].mesh = &mesh;
            delayed_direct_responses[i].contact = from;
            strncpy(delayed_direct_responses[i].response, response, sizeof(delayed_direct_responses[i].response) - 1);
            delayed_direct_responses[i].response[sizeof(delayed_direct_responses[i].response) - 1] = '\0';
            delayed_direct_responses[i].send_time = millis() + delay_ms;
            delayed_direct_responses[i].is_active = true;
            direct_response_count++;
            break;
        }
    }
}

bool TimeSyncMonitor::generateShameListMessage(char* output_buffer, size_t buffer_size) {
    if (!output_buffer || buffer_size < 64) {
        return false;
    }
    
    // Build shame list with newlines
    int pos = snprintf(output_buffer, buffer_size, "NoTimeSyncShameList:");
    
    bool has_offenders = false;
    for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
        if (repeater_states[i].is_active && repeater_states[i].on_shame_list) {
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
        snprintf(output_buffer, buffer_size, "NoTimeSyncShameList:\n(empty - all repeaters in sync!)");
        return true;
    }
    
    output_buffer[pos] = '\0';
    return true;
}

void TimeSyncMonitor::checkAndSendDailyReport(BaseChatMesh& mesh, uint32_t current_time) {
    if (!public_channel) {
        return;
    }
    
    // Only report if our own node is synced
    if (!isNodeTimeSynced(current_time)) {
        return;
    }
    
    // Apply timezone offset to get local time (automatically handles DST)
    int timezone_offset_hours = getBudapestTimezoneOffset(current_time);
    uint32_t local_time = current_time + (timezone_offset_hours * 3600);
    
    // Calculate current day (days since epoch in local timezone)
    uint32_t current_day = local_time / 86400;
    
    // Calculate current hour and minute in local timezone
    uint32_t seconds_today = local_time % 86400;
    uint32_t current_hour = seconds_today / 3600;
    uint32_t current_minute = (seconds_today % 3600) / 60;
    
    // Check if it's report time and we haven't reported today
    if (current_hour == DAILY_REPORT_HOUR && current_minute == DAILY_REPORT_MINUTE && current_day != last_report_day) {
        last_report_day = current_day;
        
        // Build shame list
        char shame_list[1024];
        int pos = snprintf(shame_list, sizeof(shame_list), "NoTimeSyncShameList:");
        
        bool has_offenders = false;
        for (int i = 0; i < MAX_TRACKED_REPEATERS; i++) {
            if (repeater_states[i].is_active && repeater_states[i].on_shame_list) {
                has_offenders = true;
                
                // Add name to list
                if (pos < (int)sizeof(shame_list) - 2) {
                    shame_list[pos++] = '\n';
                }
                
                int name_len = strlen(repeater_states[i].name);
                if (pos + name_len < (int)sizeof(shame_list) - 1) {
                    strcpy(&shame_list[pos], repeater_states[i].name);
                    pos += name_len;
                }
            }
        }
        
        if (has_offenders) {
            shame_list[pos] = '\0';
            
            // Split into multiple messages if needed (max 150 chars each)
            char current_msg[256];
            int msg_start = 0;
            int msg_pos = 0;
            
            // First message starts with "NoTimeSyncShameList:"
            strncpy(current_msg, "NoTimeSyncShameList:", sizeof(current_msg) - 1);
            msg_pos = strlen(current_msg);
            
            // Skip the header in shame_list
            int list_pos = strlen("NoTimeSyncShameList:");
            
            // Process each name
            while (list_pos < pos) {
                char c = shame_list[list_pos];
                
                if (c == '\n') {
                    // Start of a new name, check if we need to start a new message
                    list_pos++;
                    
                    // Find next newline or end
                    int next_newline = list_pos;
                    while (next_newline < pos && shame_list[next_newline] != '\n') {
                        next_newline++;
                    }
                    int name_len = next_newline - list_pos;
                    
                    // Check if adding this name would exceed limit
                    if (msg_pos + 2 + name_len > MAX_MESSAGE_LENGTH) {
                        // Send current message and start new one
                        current_msg[msg_pos] = '\0';
                        sendShameListMessage(mesh, current_msg);
                        
                        // Start new message (continuation doesn't need header)
                        msg_pos = 0;
                    }
                    
                    // Add comma separator if not first name in message
                    if (msg_pos > 0 && msg_pos < (int)sizeof(current_msg) - 2) {
                        current_msg[msg_pos++] = ',';
                        current_msg[msg_pos++] = ' ';
                    }
                    
                    // Add name
                    if (msg_pos + name_len < (int)sizeof(current_msg) - 1) {
                        memcpy(&current_msg[msg_pos], &shame_list[list_pos], name_len);
                        msg_pos += name_len;
                        list_pos += name_len;
                    }
                } else {
                    list_pos++;
                }
            }
            
            // Send final message
            if (msg_pos > 0) {
                current_msg[msg_pos] = '\0';
                sendShameListMessage(mesh, current_msg);
            }
        }
    }
    
    // Process delayed channel messages
    for (int i = 0; i < 10; i++) {
        if (delayed_messages[i].is_active && millis() >= delayed_messages[i].send_time) {
            // Send the message
            uint8_t temp[5 + MAX_TEXT_LEN + 32];
            uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
            memcpy(temp, &timestamp, 4);
            temp[4] = 0;  // attempt and flags
            
            const char* node_name = "TimeSync";  // Use a generic name for the bot
            snprintf((char*)&temp[5], MAX_TEXT_LEN, "%s: %s", node_name, delayed_messages[i].message);
            temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long
            
            int len = strlen((char*)&temp[5]);
            auto pkt = mesh.createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, *public_channel, temp, 5 + len);
            if (pkt) {
                mesh.sendFlood(pkt);
            }
            
            delayed_messages[i].is_active = false;
            delayed_message_count--;
        }
    }
    
    // Process delayed direct responses
    for (int i = 0; i < 5; i++) {
        if (delayed_direct_responses[i].is_active && millis() >= delayed_direct_responses[i].send_time) {
            uint32_t expected_ack, est_timeout;
            delayed_direct_responses[i].mesh->sendMessage(
                delayed_direct_responses[i].contact, 
                delayed_direct_responses[i].mesh->getRTCClock()->getCurrentTime(), 
                0, 
                delayed_direct_responses[i].response, 
                expected_ack, 
                est_timeout);
            
            delayed_direct_responses[i].is_active = false;
            direct_response_count--;
        }
    }
}

#endif // TIMESYNC_MONITOR_ENABLED

