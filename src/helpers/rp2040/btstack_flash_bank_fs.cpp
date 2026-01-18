/*
 * BTstack Flash Bank Implementation for MeshCore RP2040
 *
 * This file overrides arduino-pico's default btstack_flash_bank.cpp which
 * stores bonding data INSIDE the firmware binary (gets erased on .uf2 flash).
 *
 * Our implementation stores bonding data at the END of the LittleFS partition,
 * which survives firmware updates (same as user data, device name, etc.).
 *
 * Flash Layout (2MB total):
 * ┌─────────────────────────────────────────┬──────────────────────────────────┐
 * │  Firmware Partition (1.5MB)             │  Filesystem Partition (0.5MB)    │
 * │  - Gets overwritten on .uf2 flash       │  - Preserved on .uf2 flash       │
 * │                                         │  ┌─────────────────────────────┐ │
 * │                                         │  │ LittleFS files              │ │
 * │                                         │  │ (grows from start)          │ │
 * │                                         │  ├─────────────────────────────┤ │
 * │                                         │  │ BTstack NVM (8KB)           │ │
 * │                                         │  │ (at end of partition)       │ │
 * │                                         │  └─────────────────────────────┘ │
 * └─────────────────────────────────────────┴──────────────────────────────────┘
 *
 * Copyright (c) 2024 MeshCore Contributors
 * Based on arduino-pico btstack_flash_bank.cpp (c) 2023 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if defined(PICO_CYW43_SUPPORTED)

#include <btstack.h>
#include <pico/btstack_flash_bank.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <string.h>
#include <Arduino.h>

// Linker-provided symbols for filesystem partition boundaries
// These are XIP-mapped addresses (0x10xxxxxx range)
extern "C" {
  extern uint8_t _FS_start;
  extern uint8_t _FS_end;
}

// XIP base address - flash is memory-mapped here on RP2040
#ifndef XIP_BASE
  #define XIP_BASE 0x10000000
#endif

// BTstack needs 2 flash banks (for wear leveling), each one sector
// RP2040 flash sector size is 4KB, so we need 8KB total
#define BTSTACK_FLASH_BANK_SIZE     FLASH_SECTOR_SIZE  // 4KB per bank
#define BTSTACK_FLASH_BANK_TOTAL    (BTSTACK_FLASH_BANK_SIZE * 2)  // 8KB total

// Calculate storage offset at runtime
// Place BTstack NVM at the END of the filesystem partition
static uint32_t get_btstack_storage_offset(void) {
  // Get filesystem end as raw flash offset (subtract XIP_BASE)
  uint32_t fs_end = (uint32_t)&_FS_end - XIP_BASE;
  // Place our 8KB storage at the end of the filesystem partition
  return fs_end - BTSTACK_FLASH_BANK_TOTAL;
}

// Debug logging (disabled by default)
#if defined(BLE_DEBUG_LOGGING) && BLE_DEBUG_LOGGING
  #define FLASH_DEBUG_PRINT(format, ...) Serial.printf("[FLASH] " format "\n", ##__VA_ARGS__)
#else
  #define FLASH_DEBUG_PRINT(...)
#endif

// ============================================================================
// hal_flash_bank_t interface implementation
// ============================================================================

static uint32_t meshcore_flash_bank_get_size(void *context) {
  (void)context;
  return BTSTACK_FLASH_BANK_SIZE;
}

static uint32_t meshcore_flash_bank_get_alignment(void *context) {
  (void)context;
  // BTstack TLV requires alignment of 1 (byte-aligned writes)
  return 1;
}

static void meshcore_flash_bank_erase(void *context, int bank) {
  (void)context;

  if (bank < 0 || bank > 1) {
    FLASH_DEBUG_PRINT("erase: invalid bank %d", bank);
    return;
  }

  uint32_t offset = get_btstack_storage_offset() + (BTSTACK_FLASH_BANK_SIZE * bank);
  FLASH_DEBUG_PRINT("erase: bank %d at offset 0x%08lX", bank, (unsigned long)offset);

#ifndef __FREERTOS
  noInterrupts();
#endif
  rp2040.idleOtherCore();
  flash_range_erase(offset, BTSTACK_FLASH_BANK_SIZE);
  rp2040.resumeOtherCore();
#ifndef __FREERTOS
  interrupts();
#endif
}

static void meshcore_flash_bank_read(void *context, int bank, uint32_t offset, uint8_t *buffer, uint32_t size) {
  (void)context;

  if (bank < 0 || bank > 1) {
    FLASH_DEBUG_PRINT("read: invalid bank %d", bank);
    return;
  }

  if (offset >= BTSTACK_FLASH_BANK_SIZE) {
    FLASH_DEBUG_PRINT("read: invalid offset %lu", (unsigned long)offset);
    return;
  }

  if ((offset + size) > BTSTACK_FLASH_BANK_SIZE) {
    FLASH_DEBUG_PRINT("read: size %lu exceeds bank", (unsigned long)size);
    return;
  }

  uint32_t flash_offset = get_btstack_storage_offset() + (BTSTACK_FLASH_BANK_SIZE * bank) + offset;

  FLASH_DEBUG_PRINT("read: bank %d offset %lu size %lu from 0x%08lX",
                    bank, (unsigned long)offset, (unsigned long)size, (unsigned long)flash_offset);

  // Flash is memory-mapped via XIP - direct read
  memcpy(buffer, (void *)(XIP_BASE + flash_offset), size);
}

static void meshcore_flash_bank_write(void *context, int bank, uint32_t offset, const uint8_t *data, uint32_t size) {
  (void)context;

  if (bank < 0 || bank > 1) {
    FLASH_DEBUG_PRINT("write: invalid bank %d", bank);
    return;
  }

  if (offset >= BTSTACK_FLASH_BANK_SIZE) {
    FLASH_DEBUG_PRINT("write: invalid offset %lu", (unsigned long)offset);
    return;
  }

  if ((offset + size) > BTSTACK_FLASH_BANK_SIZE) {
    FLASH_DEBUG_PRINT("write: size %lu exceeds bank", (unsigned long)size);
    return;
  }

  if (size == 0) {
    return;
  }

  uint32_t bank_start = get_btstack_storage_offset() + (BTSTACK_FLASH_BANK_SIZE * bank);

  FLASH_DEBUG_PRINT("write: bank %d offset %lu size %lu to 0x%08lX",
                    bank, (unsigned long)offset, (unsigned long)size, (unsigned long)(bank_start + offset));

  // Flash writes must be page-aligned (256 bytes on RP2040)
  // We need to read-modify-write for partial page updates

  const uint32_t first_page = offset / FLASH_PAGE_SIZE;
  const uint32_t last_page = (offset + size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  // Offset within the first page
  uint32_t page_offset = offset % FLASH_PAGE_SIZE;
  uint32_t data_pos = 0;
  uint32_t size_left = size;

  for (uint32_t page = first_page; page < last_page; page++) {
    uint8_t page_data[FLASH_PAGE_SIZE];
    uint32_t page_flash_offset = bank_start + (page * FLASH_PAGE_SIZE);

    // Copy existing data for parts we're not overwriting

    // First page: preserve data before our write offset
    if (page == first_page && page_offset > 0) {
      memcpy(page_data, (void *)(XIP_BASE + page_flash_offset), page_offset);
    }

    // Last page: preserve data after our write ends
    if (page == last_page - 1 && (page_offset + size_left) < FLASH_PAGE_SIZE) {
      uint32_t preserve_offset = page_offset + size_left;
      memcpy(page_data + preserve_offset,
             (void *)(XIP_BASE + page_flash_offset + preserve_offset),
             FLASH_PAGE_SIZE - preserve_offset);
    }

    // Copy our data into the page buffer
    uint32_t copy_size = FLASH_PAGE_SIZE - page_offset;
    if (copy_size > size_left) {
      copy_size = size_left;
    }
    memcpy(page_data + page_offset, data + data_pos, copy_size);

    data_pos += copy_size;
    size_left -= copy_size;
    page_offset = 0;  // Only first page has non-zero offset

    // Program the page
#ifndef __FREERTOS
    noInterrupts();
#endif
    rp2040.idleOtherCore();
    flash_range_program(page_flash_offset, page_data, FLASH_PAGE_SIZE);
    rp2040.resumeOtherCore();
#ifndef __FREERTOS
    interrupts();
#endif
  }
}

// ============================================================================
// Flash bank instance - this is what BTstack calls
// ============================================================================

static const hal_flash_bank_t meshcore_flash_bank_instance_obj = {
  .get_size      = &meshcore_flash_bank_get_size,
  .get_alignment = &meshcore_flash_bank_get_alignment,
  .erase         = &meshcore_flash_bank_erase,
  .read          = &meshcore_flash_bank_read,
  .write         = &meshcore_flash_bank_write,
};

// This function overrides arduino-pico's pico_flash_bank_instance()
// The linker should pick this one because it's in the user's project
extern "C" {
  const hal_flash_bank_t *pico_flash_bank_instance(void) {
    FLASH_DEBUG_PRINT("pico_flash_bank_instance() called - using MeshCore filesystem storage");
    FLASH_DEBUG_PRINT("Storage offset: 0x%08lX (in filesystem partition)",
                      (unsigned long)get_btstack_storage_offset());
    return &meshcore_flash_bank_instance_obj;
  }
}

#endif // PICO_CYW43_SUPPORTED
