#include "node_online_monitor.h"
#include <string.h>

#ifdef NODE_ONLINE_MONITOR_ENABLED

// Public keys for the two monitored nodes (32 bytes each)
static const uint8_t CSOVYMPUS_PUB_KEY[32] = {
    0xC5, 0x04, 0x10, 0x07, 0x3E, 0x88, 0x7C, 0x6C,
    0xD7, 0x4A, 0xB6, 0x62, 0x2F, 0xA8, 0x61, 0xF7,
    0xD0, 0xF4, 0x21, 0x6E, 0x1E, 0xDA, 0x19, 0x28,
    0xE3, 0xA2, 0x16, 0x55, 0x61, 0xCC, 0x7A, 0x39
};

static const uint8_t NASZALY_PUB_KEY[32] = {
    0xC1, 0x3E, 0xC9, 0x34, 0x2E, 0xB7, 0xF5, 0x67,
    0xB2, 0x21, 0x82, 0x76, 0x4C, 0x24, 0x09, 0x08,
    0xDC, 0x7C, 0xE2, 0xA4, 0x92, 0x20, 0x9B, 0x5C,
    0x1A, 0x31, 0x75, 0x82, 0x0A, 0x3B, 0x2D, 0xFE
};

#define DOWNTIME_THRESHOLD_SECONDS (26 * 3600)  // 26 hours (accounting for delays)
#define ONE_MONTH_SECONDS (30 * 24 * 3600)      // 30 days
#define MESSAGE_DELAY_MS 2000

struct NodeState {
    uint8_t public_key[32];
    uint32_t last_advert_timestamp;      // Timestamp from the last advert received
    uint32_t previous_advert_timestamp;  // Timestamp from the advert before that
    uint32_t last_received_time;        // Our local time when we received the last advert
    bool is_active;
};

static NodeState node_states[2];
static const mesh::GroupChannel* hungary_channel = nullptr;

void NodeOnlineMonitor::begin() {
    memset(node_states, 0, sizeof(node_states));
    
    // Initialize Csovympus state
    memcpy(node_states[0].public_key, CSOVYMPUS_PUB_KEY, 32);
    node_states[0].is_active = false;
    node_states[0].last_advert_timestamp = 0;
    node_states[0].previous_advert_timestamp = 0;
    node_states[0].last_received_time = 0;
    
    // Initialize Naszály state
    memcpy(node_states[1].public_key, NASZALY_PUB_KEY, 32);
    node_states[1].is_active = false;
    node_states[1].last_advert_timestamp = 0;
    node_states[1].previous_advert_timestamp = 0;
    node_states[1].last_received_time = 0;
}

bool NodeOnlineMonitor::isTargetNode(const uint8_t* pub_key) {
    if (!pub_key) return false;
    return (memcmp(pub_key, CSOVYMPUS_PUB_KEY, 32) == 0) ||
           (memcmp(pub_key, NASZALY_PUB_KEY, 32) == 0);
}

const char* NodeOnlineMonitor::getNodeDisplayName(const uint8_t* pub_key) {
    if (memcmp(pub_key, CSOVYMPUS_PUB_KEY, 32) == 0) {
        return "Csovympus";
    } else if (memcmp(pub_key, NASZALY_PUB_KEY, 32) == 0) {
        return "Eye of Naszály";
    }
    return nullptr;
}

bool NodeOnlineMonitor::checkSignificantDowntime(uint32_t current_advert_time, uint32_t previous_advert_time, uint32_t last_received_time, uint32_t node_current_time) {
    // Check condition 1: Last advert received later than 26 hours ago
    // (they send adverts every 12 hours, so 26 hours indicates significant delay)
    if (last_received_time > 0) {
        uint32_t time_since_last_received;
        if (node_current_time >= last_received_time) {
            time_since_last_received = node_current_time - last_received_time;
        } else {
            // Handle time wrap-around
            time_since_last_received = (0xFFFFFFFF - last_received_time) + node_current_time + 1;
        }
        
        if (time_since_last_received > DOWNTIME_THRESHOLD_SECONDS) {
            return true;
        }
    }
    
    // Check condition 2: Last advert has time in the past more than 1 month
    // (earlier than the advert before that)
    if (previous_advert_time > 0 && current_advert_time > 0) {
        if (current_advert_time < previous_advert_time) {
            uint32_t time_diff = previous_advert_time - current_advert_time;
            if (time_diff > ONE_MONTH_SECONDS) {
                return true;
            }
        }
    }
    
    return false;
}

void NodeOnlineMonitor::sendOnlineMessage(BaseChatMesh& mesh, const char* bot_node_name, const uint8_t* pub_key) {
    if (!hungary_channel || !bot_node_name || !pub_key) {
        return;
    }
    
    const char* message = nullptr;
    if (memcmp(pub_key, CSOVYMPUS_PUB_KEY, 32) == 0) {
        message = "Csóvi online, nemzetközi kommunikáció helyreállítva.";
    } else if (memcmp(pub_key, NASZALY_PUB_KEY, 32) == 0) {
        message = "Naszály online, nemzetközi kommunikáció helyreállítva.";
    } else {
        return; // Unknown node
    }
    
    // Use staggered delays to prevent simultaneous messages
    static uint8_t message_counter = 0;
    uint32_t delay = MESSAGE_DELAY_MS + (message_counter * 1000);
    message_counter = (message_counter + 1) % 10;
    
    // Try to use enhanced channel message sending if available
    if (mesh.sendChannelMessage(*hungary_channel, message, bot_node_name, delay)) {
        MESH_DEBUG_PRINTLN("[NODE-ONLINE] Using enhanced channel message sending");
        return;
    }
    
    // Fallback to regular channel message sending
    uint8_t temp[5+MAX_TEXT_LEN+32];
    uint32_t timestamp = mesh.getRTCClock()->getCurrentTime();
    memcpy(temp, &timestamp, 4);
    temp[4] = 0;  // TXT_TYPE_PLAIN
    
    // Format: <sender_name>: <message>
    sprintf((char *) &temp[5], "%s: %s", bot_node_name, message);
    temp[5 + MAX_TEXT_LEN] = 0;  // truncate if too long
    
    int len = strlen((char *) &temp[5]);
    auto pkt = mesh.createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, *hungary_channel, temp, 5 + len);
    if (!pkt) {
        MESH_DEBUG_PRINTLN("[NODE-ONLINE] ERROR: Failed to create group datagram (packet pool full?)");
        return;
    }
    
    MESH_DEBUG_PRINTLN("[NODE-ONLINE] Using regular channel message sending");
    mesh.sendFlood(pkt, delay);
}

void NodeOnlineMonitor::setHungaryChannel(const mesh::GroupChannel* channel) {
    hungary_channel = channel;
}

void NodeOnlineMonitor::processAdvertisement(BaseChatMesh& mesh, const mesh::Packet* packet, const mesh::Identity& id, 
                                            uint32_t advertised_timestamp, const char* node_name, uint32_t node_current_time, const char* bot_node_name) {
    if (!packet || !node_name || strlen(node_name) == 0) {
        return;
    }
    
    // Check if this is one of our target nodes
    if (!isTargetNode(id.pub_key)) {
        return;
    }
    
    // Find the node state
    NodeState* state = nullptr;
    for (int i = 0; i < 2; i++) {
        if (memcmp(node_states[i].public_key, id.pub_key, 32) == 0) {
            state = &node_states[i];
            break;
        }
    }
    
    if (!state) {
        return; // Should not happen, but safety check
    }
    
    // Check if this is a significant downtime recovery
    bool had_downtime = false;
    if (state->is_active) {
        // We have previous state, check for downtime
        had_downtime = checkSignificantDowntime(advertised_timestamp, state->last_advert_timestamp, state->last_received_time, node_current_time);
    } else {
        // First advert after clean slate - check if advertised timestamp is significantly in the past
        // This indicates the node was offline and just came back
        if (advertised_timestamp > 0 && node_current_time > advertised_timestamp) {
            uint32_t time_diff = node_current_time - advertised_timestamp;
            if (time_diff > DOWNTIME_THRESHOLD_SECONDS) {
                // Advertised timestamp is more than 26 hours in the past
                // This suggests the node was offline and just came back
                had_downtime = true;
            }
        }
    }
    
    // Update state: shift timestamps
    state->previous_advert_timestamp = state->last_advert_timestamp;
    state->last_advert_timestamp = advertised_timestamp;
    state->last_received_time = node_current_time;
    state->is_active = true;
    
    // If significant downtime was detected, send message
    if (had_downtime && hungary_channel && bot_node_name) {
        sendOnlineMessage(mesh, bot_node_name, id.pub_key);
        const char* display_name = getNodeDisplayName(id.pub_key);
        if (display_name) {
            MESH_DEBUG_PRINTLN("[NODE-ONLINE] Detected downtime recovery for %s", display_name);
        }
    }
}

#endif // NODE_ONLINE_MONITOR_ENABLED

