#include "kv_db.h"
#include "hal/storage_arduino_eeprom.h"
#include "hal/storage_avr_eeprom.h"
#include "hal/storage_driver.h"
#include <string.h>

// Global state
static Mutex kv_mutex;

// SRAM cache for the index
static struct {
  uint8_t count;                   // Current number of keys
  IndexEntry entries[KV_MAX_KEYS]; // Sorted array of record addresses
  uint16_t next_free_addr;         // Next allocation address
} index_cache;

/**
 * Read a key from EEPROM at the given record address.
 * Returns the key in the provided buffer (must be at least KV_MAX_KEY_LEN+1).
 */
static void read_key_from_record(uint16_t addr, char *key_buffer) {
  KVRecord header;
  storage_read(addr, &header, sizeof(KVRecord));

  uint8_t read_len =
      (header.key_len > KV_MAX_KEY_LEN) ? KV_MAX_KEY_LEN : header.key_len;
  storage_read(addr + sizeof(KVRecord), key_buffer, read_len);
  key_buffer[read_len] = '\0';
}

/**
 * Compare two keys: one in the provided string, one at an EEPROM address.
 * Returns: <0 if key1 < key2, 0 if equal, >0 if key1 > key2
 */
static int8_t compare_keys(const char *key1, uint16_t addr2) {
  char key2[KV_MAX_KEY_LEN + 1];
  read_key_from_record(addr2, key2);
  return strcmp(key1, key2);
}

/**
 * Binary search in the sorted index.
 * Returns the index position if found, or -1 if not found.
 */
static int16_t index_search(const char *key) {
  int16_t left = 0;
  int16_t right = index_cache.count - 1;

  while (left <= right) {
    int16_t mid = left + (right - left) / 2;
    int8_t cmp = compare_keys(key, index_cache.entries[mid].addr);

    if (cmp == 0) {
      return mid; // Found
    } else if (cmp < 0) {
      right = mid - 1;
    } else {
      left = mid + 1;
    }
  }

  return -1; // Not found
}

/**
 * Find insertion point for a new key using binary search.
 * Returns the index where the key should be inserted.
 */
static uint8_t find_insert_position(const char *key) {
  if (index_cache.count == 0) {
    return 0;
  }

  int16_t left = 0;
  int16_t right = index_cache.count - 1;

  while (left <= right) {
    int16_t mid = left + (right - left) / 2;
    int8_t cmp = compare_keys(key, index_cache.entries[mid].addr);

    if (cmp < 0) {
      right = mid - 1;
    } else if (cmp > 0) {
      left = mid + 1;
    } else {
      return mid; // Key already exists
    }
  }

  return left; // Insert position
}

/**
 * Insert a new entry into the sorted index.
 * Shifts existing entries to make room.
 */
static void index_insert(uint8_t pos, uint16_t addr) {
  // Bounds check (defensive programming)
  if (index_cache.count >= KV_MAX_KEYS) {
    return; // Should never happen if callers validate properly
  }

  // Shift entries to the right
  for (uint8_t i = index_cache.count; i > pos; i--) {
    index_cache.entries[i] = index_cache.entries[i - 1];
  }

  // Insert new entry
  index_cache.entries[pos].addr = addr;
  index_cache.count++;
}

/**
 * Remove an entry from the index.
 * Shifts entries to fill the gap.
 */
static void index_remove(uint8_t pos) {
  // Shift entries to the left
  for (uint8_t i = pos; i < index_cache.count - 1; i++) {
    index_cache.entries[i] = index_cache.entries[i + 1];
  }

  index_cache.count--;
}

/**
 * Write the index to EEPROM.
 */
static void write_index_to_eeprom(void) {
  KVHeader header;
  header.magic = KV_MAGIC;
  header.count = index_cache.count;
  header.reserved = 0;

  storage_write(KV_HEADER_OFFSET, &header, sizeof(KVHeader));
  storage_write(KV_INDEX_OFFSET, index_cache.entries,
                index_cache.count * sizeof(IndexEntry));
}

/**
 * Initialize the KV database.
 * Loads the index from EEPROM or initializes a new one.
 */
kv_result_t kv_init(void) {
  os_mutex_init(&kv_mutex);

  /* Initialize storage driver if not set */
  if (storage_get_capacity() == 0) {
#ifdef __AVR__
    storage_set_driver(storage_get_avr_eeprom_driver());
#elif defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_UNOR4_MINIMA) ||        \
    defined(ARDUINO_UNOR4_WIFI)
    storage_set_driver(storage_get_arduino_eeprom_driver());
#endif
    storage_init();
  }

  // Read header
  KVHeader header;
  storage_read(KV_HEADER_OFFSET, &header, sizeof(KVHeader));

  if (header.magic == KV_MAGIC && header.count <= KV_MAX_KEYS) {
    // Valid database exists - load index
    index_cache.count = header.count;
    storage_read(KV_INDEX_OFFSET, index_cache.entries,
                 header.count * sizeof(IndexEntry));

    // Find next free address by scanning records
    index_cache.next_free_addr = KV_RECORDS_OFFSET;
    for (uint8_t i = 0; i < index_cache.count; i++) {
      uint16_t addr = index_cache.entries[i].addr;

      // Validate address is within EEPROM bounds
      if (addr < KV_RECORDS_OFFSET || addr >= KV_EEPROM_SIZE) {
        // Corrupted index - reinitialize
        index_cache.count = 0;
        index_cache.next_free_addr = KV_RECORDS_OFFSET;
        write_index_to_eeprom();
        return KV_SUCCESS;
      }

      KVRecord rec;
      storage_read(addr, &rec, sizeof(KVRecord));

      // Validate record fields to prevent overflow
      if (rec.key_len > KV_MAX_KEY_LEN || rec.val_len > KV_MAX_VAL_LEN) {
        // Corrupted record - reinitialize
        index_cache.count = 0;
        index_cache.next_free_addr = KV_RECORDS_OFFSET;
        write_index_to_eeprom();
        return KV_SUCCESS;
      }

      uint16_t record_end = addr + sizeof(KVRecord) + rec.key_len + rec.val_len;
      if (record_end > index_cache.next_free_addr) {
        index_cache.next_free_addr = record_end;
      }
    }
  } else {
    // Initialize new database
    index_cache.count = 0;
    index_cache.next_free_addr = KV_RECORDS_OFFSET;
    write_index_to_eeprom();
  }

  return KV_SUCCESS;
}

/**
 * Write a key-value pair.
 * Updates existing key or creates new entry.
 */
kv_result_t kv_write(const char *key, const char *value, uint16_t val_len) {
  if (key == NULL || value == NULL)
    return KV_ERR_EEPROM;

  uint8_t key_len = strlen(key);
  if (key_len > KV_MAX_KEY_LEN)
    return KV_ERR_KEY_TOO_LONG;
  if (val_len > KV_MAX_VAL_LEN)
    return KV_ERR_VAL_TOO_LONG;

  os_mutex_lock(&kv_mutex);

  // Check if space is available for new record
  uint16_t record_size = sizeof(KVRecord) + key_len + val_len;
  if (index_cache.next_free_addr + record_size > KV_EEPROM_SIZE) {
    os_mutex_unlock(&kv_mutex);
    return KV_ERR_FULL;
  }

  // Check if key exists
  int16_t idx = index_search(key);
  bool is_update = (idx >= 0);

  // If updating, mark old record as free
  if (is_update) {
    uint16_t old_addr = index_cache.entries[idx].addr;
    uint8_t flags = KV_FLAG_FREE;
    // Write flag to EEPROM (offset 3 in KVRecord struct)
    storage_write(old_addr + 3, &flags, sizeof(uint8_t));
  }

  // Write new record
  uint16_t addr = index_cache.next_free_addr;
  KVRecord header;
  header.key_len = key_len;
  header.val_len = val_len;
  header.flags = 0;

  storage_write(addr, &header, sizeof(KVRecord));
  storage_write(addr + sizeof(KVRecord), key, key_len);
  storage_write(addr + sizeof(KVRecord) + key_len, value, val_len);

  // Update index
  if (is_update) {
    // Replace existing entry address
    index_cache.entries[idx].addr = addr;
  } else {
    // Insert new entry
    if (index_cache.count >= KV_MAX_KEYS) {
      os_mutex_unlock(&kv_mutex);
      return KV_ERR_FULL;
    }

    uint8_t insert_pos = find_insert_position(key);
    index_insert(insert_pos, addr);
  }

  // Update next free address
  index_cache.next_free_addr += record_size;

  // Persist index to EEPROM
  write_index_to_eeprom();

  os_mutex_unlock(&kv_mutex);
  return KV_SUCCESS;
}

/**
 * Read a value by key.
 */
kv_result_t kv_read(const char *key, char *buffer, uint16_t max_len,
                    uint16_t *actual_len) {
  if (key == NULL || buffer == NULL)
    return KV_ERR_EEPROM;

  os_mutex_lock(&kv_mutex);

  // Binary search for key
  int16_t idx = index_search(key);

  if (idx < 0) {
    os_mutex_unlock(&kv_mutex);
    return KV_ERR_NOT_FOUND;
  }

  // Read record
  uint16_t addr = index_cache.entries[idx].addr;
  KVRecord header;
  storage_read(addr, &header, sizeof(KVRecord));

  // Read value
  uint16_t read_len = (header.val_len > max_len) ? max_len : header.val_len;
  uint16_t val_offset = addr + sizeof(KVRecord) + header.key_len;
  storage_read(val_offset, buffer, read_len);

  if (actual_len)
    *actual_len = header.val_len;

  os_mutex_unlock(&kv_mutex);
  return KV_SUCCESS;
}

/**
 * Delete a key.
 */
kv_result_t kv_delete(const char *key) {
  if (key == NULL)
    return KV_ERR_EEPROM;

  os_mutex_lock(&kv_mutex);

  int16_t idx = index_search(key);

  if (idx < 0) {
    os_mutex_unlock(&kv_mutex);
    return KV_ERR_NOT_FOUND;
  }

  // Remove from index (don't reclaim EEPROM space)
  index_remove(idx);

  // Persist index
  write_index_to_eeprom();

  os_mutex_unlock(&kv_mutex);
  return KV_SUCCESS;
}

/**
 * Compact the database to reclaim EEPROM space.
 * Copies all live entries to fresh EEPROM space.
 */
int16_t kv_compact(void) {
  os_mutex_lock(&kv_mutex);

  uint16_t old_next_free = index_cache.next_free_addr;
  uint16_t write_addr = KV_RECORDS_OFFSET;

  // Temporary buffer for copying record data
  char temp_buffer[KV_MAX_KEY_LEN + 1];

  // Process each entry in the index
  for (uint8_t i = 0; i < index_cache.count; i++) {
    uint16_t old_addr = index_cache.entries[i].addr;

    // Read the record header
    KVRecord header;
    storage_read(old_addr, &header, sizeof(KVRecord));

    // Calculate record size
    uint16_t record_size = sizeof(KVRecord) + header.key_len + header.val_len;

    // Check if we have space (should always succeed for compaction)
    if (write_addr + record_size > KV_EEPROM_SIZE) {
      // This shouldn't happen - the data should always fit
      os_mutex_unlock(&kv_mutex);
      return KV_ERR_FULL;
    }

    // If the record is already at the target location, skip copying
    if (old_addr == write_addr) {
      write_addr += record_size;
      continue;
    }

    // Copy the entire record (header + key + value) to new location
    // We do this in chunks to avoid using too much SRAM

    // Copy header
    storage_write(write_addr, &header, sizeof(KVRecord));

    // Copy key
    uint16_t src_offset = old_addr + sizeof(KVRecord);
    uint16_t dst_offset = write_addr + sizeof(KVRecord);
    storage_read(src_offset, temp_buffer, header.key_len);
    storage_write(dst_offset, temp_buffer, header.key_len);

    // Copy value in chunks to avoid large SRAM usage
    uint16_t val_remaining = header.val_len;
    src_offset += header.key_len;
    dst_offset += header.key_len;

    while (val_remaining > 0) {
      uint16_t chunk_size =
          (val_remaining > KV_MAX_KEY_LEN) ? KV_MAX_KEY_LEN : val_remaining;
      storage_read(src_offset, temp_buffer, chunk_size);
      storage_write(dst_offset, temp_buffer, chunk_size);

      src_offset += chunk_size;
      dst_offset += chunk_size;
      val_remaining -= chunk_size;
    }

    // Update index entry with new address
    index_cache.entries[i].addr = write_addr;

    // Move write pointer forward
    write_addr += record_size;
  }

  // Update next free address
  index_cache.next_free_addr = write_addr;

  // Persist updated index to EEPROM
  write_index_to_eeprom();

  // Calculate bytes reclaimed
  int16_t reclaimed = old_next_free - write_addr;

  os_mutex_unlock(&kv_mutex);
  return reclaimed;
}

/**
 * Clear the entire database.
 */
kv_result_t kv_clear(void) {
  os_mutex_lock(&kv_mutex);

  // Reset index
  index_cache.count = 0;
  index_cache.next_free_addr = KV_RECORDS_OFFSET;

  // Write empty header
  write_index_to_eeprom();

  os_mutex_unlock(&kv_mutex);
  return KV_SUCCESS;
}
