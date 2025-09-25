#pragma once

#include <Mesh.h>
#include <helpers/BaseChatMesh.h>

/**
 * \brief Ping-Pong helper for automatic ping/pong responses
 * 
 * This helper watches for messages containing "ping" (case insensitive)
 * and automatically responds with "pong" including hop count and router IDs.
 * 
 * Usage: Define PINGPONG_ENABLED at build time to enable this functionality.
 */

#ifdef PINGPONG_ENABLED

class PingPongHelper {
public:
    /**
     * \brief Check if a message contains "ping" and should trigger a pong response
     * \param text The message text to check
     * \returns true if the message contains "ping" with proper spacing
     */
    static bool isPingMessage(const char* text);
    
    /**
     * \brief Generate a pong response message
     * \param sender_name The name of the sender to mention
     * \param hop_count Number of hops the ping took
     * \param router_ids Comma-separated router IDs from the path
     * \param snr Signal-to-noise ratio (for direct messages)
     * \param rssi Received signal strength indicator (for direct messages)
     * \param response_buffer Buffer to store the response (must be at least 128 bytes)
     * \param buffer_size Size of the response buffer
     * \returns true if response was generated successfully
     */
    static bool generatePongResponse(const char* sender_name, uint8_t hop_count, 
                                   const char* router_ids, float snr, float rssi, char* response_buffer, size_t buffer_size);
    
    /**
     * \brief Extract hop count and router IDs from a packet path
     * \param packet The packet to extract path information from
     * \param hop_count Output parameter for hop count
     * \param router_ids_buffer Buffer to store comma-separated router IDs (must be at least 64 bytes)
     * \param buffer_size Size of the router_ids_buffer
     * \returns true if path information was extracted successfully
     */
    static bool extractPathInfo(const mesh::Packet* packet, uint8_t& hop_count, 
                               char* router_ids_buffer, size_t buffer_size);
    
    /**
     * \brief Process a received message and send pong response if needed
     * \param mesh Reference to the BaseChatMesh instance
     * \param from Contact info of the sender
     * \param packet The received packet
     * \param sender_timestamp Timestamp from sender
     * \param text The message text
     * \returns true if a pong response was sent
     */
    static bool processMessage(BaseChatMesh& mesh, const ContactInfo& from, 
                             mesh::Packet* packet, uint32_t sender_timestamp, const char* text);
};

#endif // PINGPONG_ENABLED
