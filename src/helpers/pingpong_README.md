# Ping-Pong Helper for MeshCore

This helper provides automatic ping-pong functionality for MeshCore applications. When enabled, it will automatically detect messages containing "ping" (case insensitive) and respond with a "pong" message including hop count and router IDs.

## Features

- **Case-insensitive detection**: Detects "ping", "PING", "Ping", etc.
- **Flexible spacing**: Works with "ping", " ping ", "ping ", " ping"
- **Hop count tracking**: Shows the number of hops the ping took
- **Router ID display**: Shows the router IDs in the path (e.g., "4d,6e,dd,aa,e4")
- **Channel support**: Works with both private messages and public channel messages
- **Build-time configurable**: Only compiled when `PINGPONG_ENABLED` is defined

## Usage

### Building with Ping-Pong Support

To enable ping-pong functionality, add `-DPINGPONG_ENABLED` to your build flags:

```ini
build_flags = 
  -DPINGPONG_ENABLED
  # ... other flags
```

### Example Build Command

```bash
pio run -e your_board -D PINGPONG_ENABLED
```

### Response Format

When a ping is detected, the helper will respond with:

```
@[SENDER_NAME] Pong! X hops (router_ids) [SNR: X.X dB, RSSI: XX dBm]
```

**Examples:**
- `@[TheWatcher] Pong! 5 hops (4d,6e,dd,aa,e4)` (multi-hop message)
- `@[Alice] Pong! 0 hops (direct) [SNR: 12.5 dB, RSSI: -85 dBm]` (direct message with SNR and RSSI)
- `@[Bob] Pong! 3 hops (a1,b2,c3)` (multi-hop message)

**Note:** SNR and RSSI values are only shown for direct messages (0 hops) as they represent the signal quality of the direct link.

### Integration

The ping-pong helper is already integrated into the `simple_web_logger` example. To use it in your own application:

1. Include the header:
```cpp
#include <helpers/pingpong.h>
```

2. Add ping-pong processing to your message handlers:
```cpp
#ifdef PINGPONG_ENABLED
if (PingPongHelper::processMessage(*this, from, pkt, sender_timestamp, text)) {
  Serial.println("   Sent pong response!");
}
#endif
```

3. For channel messages, you can also check manually:
```cpp
#ifdef PINGPONG_ENABLED
if (PingPongHelper::isPingMessage(text)) {
  // Handle ping in channel message
  uint8_t hop_count;
  char router_ids[64];
  if (PingPongHelper::extractPathInfo(pkt, hop_count, router_ids, sizeof(router_ids))) {
    char response[128];
    if (PingPongHelper::generatePongResponse(sender_name, hop_count, router_ids, response, sizeof(response))) {
      // Send response
    }
  }
}
#endif
```

## API Reference

### `PingPongHelper::isPingMessage(const char* text)`
Checks if a message contains "ping" with proper spacing.

### `PingPongHelper::generatePongResponse(const char* sender_name, uint8_t hop_count, const char* router_ids, float snr, float rssi, char* response_buffer, size_t buffer_size)`
Generates a formatted pong response message with SNR and RSSI information for direct messages.

### `PingPongHelper::extractPathInfo(const mesh::Packet* packet, uint8_t& hop_count, char* router_ids_buffer, size_t buffer_size)`
Extracts hop count and router IDs from a packet's path information.

### `PingPongHelper::processMessage(BaseChatMesh& mesh, const ContactInfo& from, mesh::Packet* packet, uint32_t sender_timestamp, const char* text)`
Processes a received message and sends pong response if needed.

## Xiao S3 WIO Integration

The ping-pong functionality has been integrated into the Xiao S3 WIO variant. To build and flash:

```bash
# Build the web logger with ping-pong support
pio run -e Xiao_S3_WIO_web_logger_pingpong

# Upload to device
pio run -e Xiao_S3_WIO_web_logger_pingpong -t upload
```

This environment includes:
- Ping-pong functionality enabled (`-DPINGPONG_ENABLED`)
- Web logging capabilities
- Support for both private messages and public channels
- SNR and RSSI display for direct messages

## Notes

- The helper only compiles when `PINGPONG_ENABLED` is defined, keeping it optional
- Router IDs are displayed as 2-character hexadecimal values
- Direct messages show "direct" instead of router IDs and include SNR and RSSI values
- Multi-hop messages show router IDs but no SNR/RSSI (as these vary per hop)
- The helper works with both private messages and public channel messages
- Channel messages extract the sender name from the "Name: message" format
- SNR values are displayed in dB with one decimal place precision
- RSSI values are displayed in dBm with no decimal places
