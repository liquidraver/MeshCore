class RawPacketParser {
  constructor() {
    this.packets = [];
    this.stats = { total: 0, byType: {}, byChannel: {}, firstHops: {}, senderHashes: {}, destinationHashes: {}, communicationPairs: {} };
    this.startTime = Date.now();
    
    this.knownChannelHashes = {
      0x11: 'Public'
    };
    
    this.typeNames = ['REQ', 'RESPONSE', 'TXT_MSG', 'ACK', 'ADVERT', 'GRP_TXT', 'GRP_DATA', 'ANON_REQ', 'PATH', 'TRACE', 'MULTIPART', 'RAW_CUSTOM'];
    this.routeNames = ['TRANSPORT_FLOOD', 'FLOOD', 'DIRECT', 'TRANSPORT_DIRECT'];
    this.UNKNOWN = 'Unknown';
    this.SETTINGS_KEY = 'meshcore_sniffer_settings';
    
    // Packet type constants from MeshCore
    this.PAYLOAD_TYPES = {
      REQ: 0x00,
      RESPONSE: 0x01,
      TXT_MSG: 0x02,
      ACK: 0x03,
      ADVERT: 0x04,
      GRP_TXT: 0x05,
      GRP_DATA: 0x06,
      ANON_REQ: 0x07,
      PATH: 0x08,
      TRACE: 0x09,
      MULTIPART: 0x0A,
      RAW_CUSTOM: 0x0F
    };
    
    this.ROUTE_TYPES = {
      TRANSPORT_FLOOD: 0x00,
      FLOOD: 0x01,
      DIRECT: 0x02,
      TRANSPORT_DIRECT: 0x03
    };
  }
  
  saveSettings() {
    const settings = {
      packets: this.packets,
      stats: this.stats,
      startTime: this.startTime,
      knownChannelHashes: this.knownChannelHashes
    };
    localStorage.setItem(this.SETTINGS_KEY, JSON.stringify(settings));
  }
  
  loadSettings() {
    try {
      const saved = localStorage.getItem(this.SETTINGS_KEY);
      if (saved) {
        const settings = JSON.parse(saved);
        this.packets = settings.packets || [];
        this.stats = settings.stats || { total: 0, byType: {}, byChannel: {}, firstHops: {} };
        this.startTime = settings.startTime || Date.now();
        
        if (settings.knownChannelHashes) {
          this.knownChannelHashes = { ...this.knownChannelHashes, ...settings.knownChannelHashes };
        }
      }
    } catch (e) {
    }
  }
  
  clear() {
    this.packets = [];
    this.stats = { total: 0, byType: {}, byChannel: {}, firstHops: {}, senderHashes: {}, destinationHashes: {}, communicationPairs: {} };
    this.startTime = Date.now();
    this.saveSettings();
  }

  async parseLine(line) {
    const parts = line.trim().split('|');
    if (parts.length < 5) return null;

    const packet = {
      timestamp: Date.now(),
      snr: parseFloat(parts[2]),
      rssi: parseFloat(parts[3]),
      length: parseInt(parts[4]),
      rawBytes: this.parseHexBytes(parts[5]),
      rawHex: parts[5]
    };

    if (packet.rawBytes.length >= 2) {
      const info = await this.parsePacketInfo(packet.rawBytes);
      packet.type = info.type;
      packet.routeType = info.routeType;
      packet.payloadType = info.payloadType;
      packet.payloadVersion = info.payloadVersion;
      packet.pathLen = info.pathLen;
      packet.channelHash = info.channelHash;
      packet.channelName = info.channelName;
      packet.path = info.path;
      packet.packetTimestamp = info.packetTimestamp;
      
      // Sender/Destination info
      packet.senderHash = info.senderHash;
      packet.destinationHash = info.destinationHash;
      packet.senderPublicKey = info.senderPublicKey;
      packet.ephemeralPublicKey = info.ephemeralPublicKey;
      
      // Additional parsed info
      packet.additionalInfo = info.additionalInfo;
    } else {
      packet.type = this.UNKNOWN;
    }

    this.updateStats(packet);
    this.packets.push(packet);
    this.saveSettings();
    return packet;
  }

  parseHexBytes(hexString) {
    const bytes = new Uint8Array(hexString.length / 2);
    for (let i = 0; i < hexString.length; i += 2) {
      const byte = parseInt(hexString.substring(i, i + 2), 16);
      if (!isNaN(byte)) bytes[i / 2] = byte;
    }
    return bytes;
  }

  addChannel(channelHash, channelName) {
    this.knownChannelHashes[channelHash] = channelName;
    this.saveSettings();
  }

  async parsePacketInfo(bytes) {
    if (bytes.length < 2) return { type: this.UNKNOWN };

    const header = bytes[0];
    const routeType = header & 0x03;
    const payloadType = (header >> 2) & 0x0F;
    const payloadVersion = (header >> 6) & 0x03;
    
    let offset = 1;
    let pathLen = 0;
    let path = [];
    let channelHash = null;
    let channelName = this.UNKNOWN;
    
    // Extract sender/destination info based on packet type
    let senderHash = null;
    let destinationHash = null;
    let senderPublicKey = null;
    let destinationPublicKey = null;
    let ephemeralPublicKey = null;
    let packetTimestamp = null;
    let additionalInfo = {};

    // Parse transport codes if present
    if ((routeType === 0 || routeType === 3) && bytes.length >= offset + 4) {
      const transportCode1 = (bytes[offset + 1] << 8) | bytes[offset];
      const transportCode2 = (bytes[offset + 3] << 8) | bytes[offset + 2];
      additionalInfo.transportCodes = [transportCode1, transportCode2];
      offset += 4;
    }

    // Parse path
    if (bytes.length >= offset + 1) {
      pathLen = bytes[offset++];
      
      if (pathLen > 0 && pathLen <= 64 && bytes.length >= offset + pathLen) {
        path = Array.from(bytes.slice(offset, offset + pathLen));
        offset += pathLen;
      } else if (pathLen > 64) {
        pathLen = 0;
      } else {
        pathLen = 0;
      }
    }

    // Parse payload based on type
    const payloadStart = offset;
    const payloadLen = bytes.length - offset;
    
    if (payloadLen > 0) {
      switch (payloadType) {
        case this.PAYLOAD_TYPES.REQ:
        case this.PAYLOAD_TYPES.RESPONSE:
        case this.PAYLOAD_TYPES.TXT_MSG:
        case this.PAYLOAD_TYPES.PATH:
          // These have: dest_hash(1) + src_hash(1) + MAC(2) + ciphertext
          if (payloadLen >= 4) {
            destinationHash = bytes[offset];
            senderHash = bytes[offset + 1];
            additionalInfo.mac = bytes.slice(offset + 2, offset + 4);
            additionalInfo.ciphertextLen = payloadLen - 4;
          }
          break;
          
        case this.PAYLOAD_TYPES.ADVERT:
          // Has: public_key(32) + timestamp(4) + signature(64) + appdata
          if (payloadLen >= 100) {
            senderPublicKey = bytes.slice(offset, offset + 32);
            senderHash = bytes[offset]; // First byte of public key
            const timestampBytes = bytes.slice(offset + 32, offset + 36);
            packetTimestamp = (timestampBytes[3] << 24) | (timestampBytes[2] << 16) | (timestampBytes[1] << 8) | timestampBytes[0];
            additionalInfo.signature = bytes.slice(offset + 36, offset + 100);
            additionalInfo.appdataLen = payloadLen - 100;
            
            // Parse appdata if present
            if (payloadLen > 100) {
              const appdataOffset = offset + 100;
              const flags = bytes[appdataOffset];
              additionalInfo.appdataFlags = flags;
              
              let appdataOffset2 = appdataOffset + 1;
              if (flags & 0x10) { // has location
                if (payloadLen >= appdataOffset2 + 8) {
                  const lat = (bytes[appdataOffset2 + 3] << 24) | (bytes[appdataOffset2 + 2] << 16) | (bytes[appdataOffset2 + 1] << 8) | bytes[appdataOffset2];
                  const lon = (bytes[appdataOffset2 + 7] << 24) | (bytes[appdataOffset2 + 6] << 16) | (bytes[appdataOffset2 + 5] << 8) | bytes[appdataOffset2 + 4];
                  additionalInfo.location = { lat: lat / 1000000, lon: lon / 1000000 };
                  appdataOffset2 += 8;
                }
              }
              if (flags & 0x20) { // has feature 1
                if (payloadLen >= appdataOffset2 + 2) {
                  additionalInfo.feature1 = (bytes[appdataOffset2 + 1] << 8) | bytes[appdataOffset2];
                  appdataOffset2 += 2;
                }
              }
              if (flags & 0x40) { // has feature 2
                if (payloadLen >= appdataOffset2 + 2) {
                  additionalInfo.feature2 = (bytes[appdataOffset2 + 1] << 8) | bytes[appdataOffset2];
                  appdataOffset2 += 2;
                }
              }
              if (flags & 0x80) { // has name
                const nameLen = payloadLen - appdataOffset2;
                if (nameLen > 0) {
                  additionalInfo.nodeName = new TextDecoder().decode(bytes.slice(appdataOffset2, appdataOffset2 + nameLen));
                }
              }
            }
          }
          break;
          
        case this.PAYLOAD_TYPES.ANON_REQ:
          // Has: dest_hash(1) + public_key(32) + MAC(2) + ciphertext
          if (payloadLen >= 35) {
            destinationHash = bytes[offset];
            ephemeralPublicKey = bytes.slice(offset + 1, offset + 33);
            additionalInfo.mac = bytes.slice(offset + 33, offset + 35);
            additionalInfo.ciphertextLen = payloadLen - 35;
          }
          break;
          
        case this.PAYLOAD_TYPES.GRP_TXT:
        case this.PAYLOAD_TYPES.GRP_DATA:
          // Has: channel_hash(1) + MAC(2) + ciphertext
          if (payloadLen >= 3) {
            channelHash = bytes[offset];
            additionalInfo.mac = bytes.slice(offset + 1, offset + 3);
            additionalInfo.ciphertextLen = payloadLen - 3;
            
            if (this.knownChannelHashes[channelHash]) {
              channelName = this.knownChannelHashes[channelHash];
            } else {
              const hexId = channelHash.toString(16).toUpperCase().padStart(2, '0');
              channelName = `${this.UNKNOWN} (${hexId})`;
              this.knownChannelHashes[channelHash] = channelName;
            }
          }
          break;
          
        case this.PAYLOAD_TYPES.ACK:
          // Has: checksum(4)
          if (payloadLen >= 4) {
            const checksum = (bytes[offset + 3] << 24) | (bytes[offset + 2] << 16) | (bytes[offset + 1] << 8) | bytes[offset];
            additionalInfo.checksum = checksum;
          }
          break;
          
        case this.PAYLOAD_TYPES.TRACE:
          // Similar to REQ/RESPONSE but with path info
          if (payloadLen >= 4) {
            destinationHash = bytes[offset];
            senderHash = bytes[offset + 1];
            additionalInfo.mac = bytes.slice(offset + 2, offset + 4);
            additionalInfo.ciphertextLen = payloadLen - 4;
          }
          break;
          
        case this.PAYLOAD_TYPES.MULTIPART:
          // TODO: Parse multipart structure
          additionalInfo.multipartData = bytes.slice(offset);
          break;
          
        case this.PAYLOAD_TYPES.RAW_CUSTOM:
          // Raw custom data
          additionalInfo.customData = bytes.slice(offset);
          break;
      }
    }

    // Build type string
    let type = this.typeNames[payloadType] || this.UNKNOWN;
    
    if (channelHash && (payloadType === this.PAYLOAD_TYPES.GRP_TXT || payloadType === this.PAYLOAD_TYPES.GRP_DATA)) {
      type = `${type} (${channelName})`;
    }
    
    if (routeType !== this.ROUTE_TYPES.DIRECT) {
      type += ` ${this.routeNames[routeType]}`;
    }

    return {
      type,
      routeType: this.routeNames[routeType],
      payloadType: this.typeNames[payloadType],
      payloadVersion,
      pathLen,
      channelHash,
      channelName,
      path: path.map(b => b.toString(16).toUpperCase().padStart(2, '0')),
      packetTimestamp,
      
      // Sender/Destination info
      senderHash: senderHash ? senderHash.toString(16).toUpperCase().padStart(2, '0') : null,
      destinationHash: destinationHash ? destinationHash.toString(16).toUpperCase().padStart(2, '0') : null,
      senderPublicKey: senderPublicKey ? Array.from(senderPublicKey).map(b => b.toString(16).toUpperCase().padStart(2, '0')).join('') : null,
      ephemeralPublicKey: ephemeralPublicKey ? Array.from(ephemeralPublicKey).map(b => b.toString(16).toUpperCase().padStart(2, '0')).join('') : null,
      
      // Additional parsed info
      additionalInfo
    };
  }

  updateStats(packet) {
    this.stats.total++;
    this.stats.byType[packet.type] = (this.stats.byType[packet.type] || 0) + 1;
    
    if (packet.channelName && (packet.payloadType === 'GRP_TXT' || packet.payloadType === 'GRP_DATA')) {
      this.stats.byChannel[packet.channelName] = (this.stats.byChannel[packet.channelName] || 0) + 1;
    }
    
    if (packet.path && packet.path.length > 0) {
      const firstHop = packet.path[0];
      this.stats.firstHops[firstHop] = (this.stats.firstHops[firstHop] || 0) + 1;
    }
    
    // Track sender hashes
    if (packet.senderHash) {
      this.stats.senderHashes[packet.senderHash] = (this.stats.senderHashes[packet.senderHash] || 0) + 1;
    }
    
    // Track destination hashes
    if (packet.destinationHash) {
      this.stats.destinationHashes[packet.destinationHash] = (this.stats.destinationHashes[packet.destinationHash] || 0) + 1;
    }
    
    // Track communication pairs (sender -> destination)
    if (packet.senderHash && packet.destinationHash) {
      const pair = `${packet.senderHash} → ${packet.destinationHash}`;
      this.stats.communicationPairs[pair] = (this.stats.communicationPairs[pair] || 0) + 1;
    }
  }

  getPackets(filter = {}) {
    if (!filter.payloadType && !filter.routeType && filter.hopCount === undefined) {
      return this.packets;
    }
    
    return this.packets.filter(packet => {
      if (filter.payloadType && packet.payloadType !== this.typeNames[filter.payloadType]) return false;
      if (filter.routeType && packet.routeType !== filter.routeType) return false;
      if (filter.hopCount !== undefined && packet.pathLen !== filter.hopCount) return false;
      return true;
    });
  }

  getStats(filter = {}) {
    const filteredPackets = this.getPackets(filter);
    
    const stats = {
      total: filteredPackets.length,
      byType: {},
      byChannel: {},
      firstHops: {},
      senderHashes: {},
      destinationHashes: {},
      communicationPairs: {}
    };
    
    filteredPackets.forEach(packet => {
      stats.byType[packet.type] = (stats.byType[packet.type] || 0) + 1;
      
      if (packet.channelName && (packet.payloadType === 'GRP_TXT' || packet.payloadType === 'GRP_DATA')) {
        stats.byChannel[packet.channelName] = (stats.byChannel[packet.channelName] || 0) + 1;
      }
      
      if (packet.path && packet.path.length > 0) {
        const firstHop = packet.path[0];
        stats.firstHops[firstHop] = (stats.firstHops[firstHop] || 0) + 1;
      }
      
      // Track sender hashes for filtered packets
      if (packet.senderHash) {
        stats.senderHashes[packet.senderHash] = (stats.senderHashes[packet.senderHash] || 0) + 1;
      }
      
      // Track destination hashes for filtered packets
      if (packet.destinationHash) {
        stats.destinationHashes[packet.destinationHash] = (stats.destinationHashes[packet.destinationHash] || 0) + 1;
      }
      
      // Track communication pairs for filtered packets
      if (packet.senderHash && packet.destinationHash) {
        const pair = `${packet.senderHash} → ${packet.destinationHash}`;
        stats.communicationPairs[pair] = (stats.communicationPairs[pair] || 0) + 1;
      }
    });
    
    return stats;
  }
}
