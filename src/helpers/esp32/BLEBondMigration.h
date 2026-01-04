#pragma once

#include <nvs.h>
#include <nvs_flash.h>
#include <string.h>
#include <stdlib.h>
#include "syscfg/syscfg.h"
#include "nimble/nimble/include/nimble/ble.h"
#include "nimble/nimble/host/include/host/ble_store.h"

/**
 * BLE Bond Migration: Bluedroid -> NimBLE
 *
 * Call migrateBluedroidBondsToNimBLE() BEFORE NimBLEDevice::init()
 * Migration only runs if there is no nimble bond already && Bluedroid has at least one
 */

namespace BLEBondMigration {

static constexpr const char* BLUEDROID_NS = "bt_config.conf";
static constexpr const char* NIMBLE_NS = "nimble_bond";
static constexpr const char* BT_CFG_KEY = "bt_cfg_key";
static constexpr const char* KEY_PENC = "LE_KEY_PENC";
static constexpr const char* KEY_PID = "LE_KEY_PID";
static constexpr const char* KEY_PCSRK = "LE_KEY_PCSRK";
static constexpr const char* KEY_LENC = "LE_KEY_LENC";
static constexpr const char* KEY_LCSRK = "LE_KEY_LCSRK";
static constexpr const char* KEY_ADDR_TYPE = "AddrType";

// Bluedroid key structures
#pragma pack(push, 1)
struct BD_PencKey { uint8_t ltk[16]; uint8_t rand[8]; uint16_t ediv; uint8_t sec_level; uint8_t key_size; };
struct BD_PcsrkKey { uint32_t counter; uint8_t csrk[16]; uint8_t sec_level; };
struct BD_PidKey { uint8_t irk[16]; uint8_t addr_type; uint8_t static_addr[6]; };
struct BD_LencKey { uint8_t ltk[16]; uint16_t div; uint8_t key_size; uint8_t sec_level; };
struct BD_LcsrkKey { uint32_t counter; uint16_t div; uint8_t sec_level; uint8_t csrk[16]; };
#pragma pack(pop)

// Check if NimBLE namespace has any bonds
inline bool nimbleHasBonds() {
    nvs_handle_t h;
    if (nvs_open(NIMBLE_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t sz = 0;
    esp_err_t err = nvs_get_blob(h, "peer_sec_1", nullptr, &sz);
    nvs_close(h);
    return (err == ESP_OK && sz > 0);
}

inline bool parseMacFromSection(const char* section, uint8_t* mac) {
    if (section[0] != '[') return false;
    unsigned int m[6];
    if (sscanf(section + 1, "%02x:%02x:%02x:%02x:%02x:%02x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
    return true;
}

inline int hexDecode(const char* hex, uint8_t* out, size_t max_len) {
    size_t len = strlen(hex);
    if (len % 2 != 0 || len / 2 > max_len) return -1;
    for (size_t i = 0; i < len / 2; i++) {
        unsigned int val;
        if (sscanf(hex + i * 2, "%02x", &val) != 1) return -1;
        out[i] = (uint8_t)val;
    }
    return len / 2;
}

inline bool writeNimbleBond(int idx, const ble_store_value_sec& sec, bool peer) {
    nvs_handle_t h;
    if (nvs_open(NIMBLE_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    char key[16];
    snprintf(key, sizeof(key), "%s_%d", peer ? "peer_sec" : "our_sec", idx);
    esp_err_t err = nvs_set_blob(h, key, &sec, sizeof(sec));
    if (err == ESP_OK) nvs_commit(h);
    nvs_close(h);
    return err == ESP_OK;
}

inline char* readBluedroidConfig() {
    nvs_handle_t h;
    if (nvs_open(BLUEDROID_NS, NVS_READONLY, &h) != ESP_OK) return nullptr;

    size_t total = 0;
    for (int i = 0; i < 10; i++) {
        char key[16];
        snprintf(key, sizeof(key), "%s%d", BT_CFG_KEY, i);
        size_t sz = 0;
        if (nvs_get_blob(h, key, nullptr, &sz) != ESP_OK) break;
        total += sz;
    }
    if (total == 0) { nvs_close(h); return nullptr; }

    char* buf = (char*)malloc(total + 1);
    if (!buf) { nvs_close(h); return nullptr; }

    size_t pos = 0;
    for (int i = 0; i < 10 && pos < total; i++) {
        char key[16];
        snprintf(key, sizeof(key), "%s%d", BT_CFG_KEY, i);
        size_t sz = total - pos;
        if (nvs_get_blob(h, key, buf + pos, &sz) != ESP_OK) break;
        pos += sz;
    }
    buf[pos] = '\0';
    nvs_close(h);
    return buf;
}

inline const char* getConfigValue(const char* config, const char* section, const char* key) {
    static char val[512];

    const char* sec = strstr(config, section);
    if (!sec) return nullptr;
    sec += strlen(section);

    const char* next_sec = strchr(sec, '[');
    size_t sec_len = next_sec ? (next_sec - sec) : strlen(sec);

    char search[64];
    snprintf(search, sizeof(search), "\n%s", key);

    const char* kv = strstr(sec, search);
    if (!kv || (size_t)(kv - sec) > sec_len) return nullptr;

    kv += strlen(search);
    while (*kv == ' ' || *kv == '\t') kv++;
    if (*kv != '=') return nullptr;
    kv++;
    while (*kv == ' ' || *kv == '\t') kv++;

    const char* end = strchr(kv, '\n');
    size_t vlen = end ? (end - kv) : strlen(kv);
    if (vlen >= sizeof(val)) vlen = sizeof(val) - 1;
    memcpy(val, kv, vlen);
    val[vlen] = '\0';

    while (vlen > 0 && (val[vlen-1] == '\r' || val[vlen-1] == ' ')) val[--vlen] = '\0';
    return val;
}

inline int migrateBluedroidBondsToNimBLE() {
    if (nimbleHasBonds()) return 0;

    char* config = readBluedroidConfig();
    if (!config) return 0;

    int migrated = 0;
    const char* p = config;

    while ((p = strchr(p, '[')) != nullptr) {
        uint8_t mac[6];
        if (parseMacFromSection(p, mac)) {
            char section[20];
            const char* sec_end = strchr(p, ']');
            if (sec_end && (sec_end - p) < (int)sizeof(section) - 1) {
                size_t sec_len = sec_end - p + 1;
                memcpy(section, p, sec_len);
                section[sec_len] = '\0';
            } else {
                snprintf(section, sizeof(section), "[%02x:%02x:%02x:%02x:%02x:%02x]",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            }

            ble_store_value_sec peer_sec;
            memset(&peer_sec, 0, sizeof(peer_sec));
            bool has_peer_keys = false;

            ble_store_value_sec our_sec;
            memset(&our_sec, 0, sizeof(our_sec));
            bool has_our_keys = false;

            const char* addr_type = getConfigValue(config, section, KEY_ADDR_TYPE);
            uint8_t addr_type_val = addr_type ? atoi(addr_type) : BLE_ADDR_PUBLIC;
            peer_sec.peer_addr.type = addr_type_val;
            memcpy(peer_sec.peer_addr.val, mac, 6);
            our_sec.peer_addr.type = addr_type_val;
            memcpy(our_sec.peer_addr.val, mac, 6);

            // PENC (peer's LTK)
            const char* penc = getConfigValue(config, section, KEY_PENC);
            if (penc) {
                BD_PencKey pk;
                if (hexDecode(penc, (uint8_t*)&pk, sizeof(pk)) == sizeof(pk)) {
                    memcpy(peer_sec.ltk, pk.ltk, 16);
                    memcpy(&peer_sec.rand_num, pk.rand, 8);
                    peer_sec.ediv = pk.ediv;
                    peer_sec.key_size = pk.key_size;
                    peer_sec.ltk_present = 1;
                    peer_sec.authenticated = (pk.sec_level >= 2) ? 1 : 0;
                    has_peer_keys = true;
                }
            }

            // LENC (our LTK)
            const char* lenc = getConfigValue(config, section, KEY_LENC);
            if (lenc) {
                BD_LencKey lk;
                if (hexDecode(lenc, (uint8_t*)&lk, sizeof(lk)) == sizeof(lk)) {
                    memcpy(our_sec.ltk, lk.ltk, 16);
                    our_sec.ediv = lk.div;
                    our_sec.rand_num = 0;
                    our_sec.key_size = lk.key_size;
                    our_sec.ltk_present = 1;
                    our_sec.authenticated = (lk.sec_level >= 2) ? 1 : 0;
                    has_our_keys = true;
                }
            }

            // PID (peer's IRK)
            const char* pid = getConfigValue(config, section, KEY_PID);
            if (pid) {
                BD_PidKey pk;
                if (hexDecode(pid, (uint8_t*)&pk, sizeof(pk)) == sizeof(pk)) {
                    memcpy(peer_sec.irk, pk.irk, 16);
                    peer_sec.irk_present = 1;
                    peer_sec.peer_addr.type = pk.addr_type;
                    memcpy(peer_sec.peer_addr.val, pk.static_addr, 6);
                    our_sec.peer_addr.type = pk.addr_type;
                    memcpy(our_sec.peer_addr.val, pk.static_addr, 6);
                    has_peer_keys = true;
                }
            }

            // PCSRK (peer's CSRK)
            const char* pcsrk = getConfigValue(config, section, KEY_PCSRK);
            if (pcsrk) {
                BD_PcsrkKey pk;
                if (hexDecode(pcsrk, (uint8_t*)&pk, sizeof(pk)) == sizeof(pk)) {
                    memcpy(peer_sec.csrk, pk.csrk, 16);
                    peer_sec.csrk_present = 1;
                    peer_sec.sign_counter = pk.counter;
                    has_peer_keys = true;
                }
            }

            // LCSRK (our CSRK)
            const char* lcsrk = getConfigValue(config, section, KEY_LCSRK);
            if (lcsrk) {
                BD_LcsrkKey lk;
                if (hexDecode(lcsrk, (uint8_t*)&lk, sizeof(lk)) == sizeof(lk)) {
                    memcpy(our_sec.csrk, lk.csrk, 16);
                    our_sec.csrk_present = 1;
                    our_sec.sign_counter = lk.counter;
                    has_our_keys = true;
                }
            }

            // Write bonds
            bool ok = true;
            if (has_peer_keys) {
                peer_sec.bond_count = migrated + 1;
                ok = writeNimbleBond(migrated + 1, peer_sec, true);
            }
            if (ok && has_our_keys) {
                our_sec.bond_count = migrated + 1;
                ok = writeNimbleBond(migrated + 1, our_sec, false);
            }
            if (ok && (has_peer_keys || has_our_keys)) {
                migrated++;
            }
        }
        p++;
    }

    free(config);
    return migrated;
}

}
