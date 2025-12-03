#pragma once

#include <Mesh.h>
#include <helpers/BaseChatMesh.h>

#ifdef NODE_ONLINE_MONITOR_ENABLED

class NodeOnlineMonitor {
public:
    static void begin();
    static void processAdvertisement(BaseChatMesh& mesh, const mesh::Packet* packet, const mesh::Identity& id, 
                                    uint32_t advertised_timestamp, const char* node_name, uint32_t node_current_time, const char* bot_node_name);
    static void setHungaryChannel(const mesh::GroupChannel* channel);
    
private:
    static bool isTargetNode(const uint8_t* pub_key);
    static bool checkSignificantDowntime(uint32_t current_advert_time, uint32_t previous_advert_time, uint32_t last_received_time, uint32_t node_current_time);
    static void sendOnlineMessage(BaseChatMesh& mesh, const char* bot_node_name, const uint8_t* pub_key);
    static const char* getNodeDisplayName(const uint8_t* pub_key);
};

#endif // NODE_ONLINE_MONITOR_ENABLED

