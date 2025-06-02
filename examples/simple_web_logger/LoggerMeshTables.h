#pragma once

#include <Mesh.h>
#include <helpers/SimpleMeshTables.h>

class LoggerMeshTables : public SimpleMeshTables {

public:
  LoggerMeshTables() { }

  bool hasSeen(const mesh::Packet* packet) override {
    // Always false. This allows onRecvPacket to be called multiple times for
    // packet received from multiple routes.
    return false;
  }

  bool hasSeen2(const mesh::Packet* packet) {
    // This is used for forward check. We don't want to forward duplicates.
    return SimpleMeshTables::hasSeen(packet);
  }
};
