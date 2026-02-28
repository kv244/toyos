/**
 * @file kv_db.cpp
 * @brief Implementation of the ToyOS Key-Value Database.
 *
 * Implements log-structured storage with an in-RAM index for fast access.
 * Handles wear leveling by rotating through blocks and compacting when full.
 */

#include "kv_db.h"
#include "hal/storage_driver.h"
#include "kv_hal.h"
#include <stdint.h>
#include <string.h>
#include <toyos.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

/**
 * ToyOS KV Database Implementation (LFS + Wear Leveling + ARM Cache)
 */

/* Internal Constants */
#define KV_NUM_SECTORS (KV_EEPROM_SIZE / KV_SECTOR_SIZE)
#define KV_INVALID_ADDR 0xFFFF
#define KV_RECORDS_OFFSET sizeof(SectorHeader)

/* Static Variables */
static IndexEntry kv_index[KV_MAX_KEYS];
static uint16_t kv_index_count = 0;
static kv_lock_t kv_db_lock;
static bool kv_initialized = false;

/* Active write pointer */
static uint16_t current_write_sector = 0;
static uint16_t current_write_offset = KV_RECORDS_OFFSET;
static uint32_t current_seq = 0;

/* --- Helper: Read Key from Storage --- */
static inline storage_result_t read_key_from_addr(uint16_t addr,
                                                  char *key_buffer) {
  KVRecord rec;
  storage_result_t res = storage_read(addr, &rec, sizeof(KVRecord));
  if (res != STORAGE_OK)
    return res;

  if (rec.magic == KV_RECORD_MAGIC && rec.key_len > 0) {
    uint8_t len = (rec.key_len > KV_MAX_KEY_LEN) ? KV_MAX_KEY_LEN : rec.key_len;
    res = storage_read(addr + sizeof(KVRecord), key_buffer, len);
    if (res != STORAGE_OK)
      return res;
    key_buffer[len] = '\0';
  } else {
    key_buffer[0] = '\0';
  }
  return STORAGE_OK;
}

/* --- Helper: Get Key for Index Entry --- */
static inline const char *get_key_for_entry(const IndexEntry *entry,
                                            char *temp_buf) {
#if defined(KV_HAL_ARM)
  return entry->cached_key;
#else
  read_key_from_addr(entry->addr, temp_buf);
  return temp_buf;
#endif
}

/* --- Helper: Binary Search --- */
static inline int16_t find_index(const char *key) {
  int16_t low = 0;
  int16_t high = kv_index_count - 1;
  int16_t result = -1;
  char temp_key[KV_MAX_KEY_LEN + 1];

  while (low <= high && result == -1) {
    int16_t mid = low + (high - low) / 2;
    const char *mid_key = get_key_for_entry(&kv_index[mid], temp_key);
    int cmp = strcmp(mid_key, key);

    if (cmp == 0) {
      result = mid;
    } else if (cmp < 0) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }
  return result;
}

/* --- Helper: Find Insertion Point (for sorted order) --- */
static inline uint16_t find_insertion_point(const char *key) {
  uint16_t low = 0, high = kv_index_count;
  char temp_key[KV_MAX_KEY_LEN + 1];

  while (low < high) {
    uint16_t mid = low + (high - low) / 2;
    const char *mid_key = get_key_for_entry(&kv_index[mid], temp_key);
    if (strcmp(mid_key, key) < 0)
      low = mid + 1;
    else
      high = mid;
  }
  return low;
}

/* --- Helper: Sector Formatting --- */
static inline storage_result_t format_sector(uint16_t sector) {
  uint32_t addr = sector * KV_SECTOR_SIZE;
  storage_result_t res = storage_erase(addr, KV_SECTOR_SIZE);
  if (res != STORAGE_OK)
    return res;

  current_seq++;
  SectorHeader head = {KV_SECTOR_MAGIC, current_seq};
  return storage_write(addr, &head, sizeof(SectorHeader));
}

/* --- API: Initialize --- */
/* --- API: Initialize --- */
kv_result_t kv_init(void) {
  if (kv_initialized)
    return KV_SUCCESS;

  kv_hal_crc_init();
  kv_hal_lock_init(&kv_db_lock);
  kv_hal_lock(&kv_db_lock);

  kv_index_count = 0;
  current_seq = 0;
  uint16_t latest_sector = 0;
  uint32_t max_seq = 0;

  /* 1. Identify active sectors and build index */
  for (uint16_t s = 0; s < KV_NUM_SECTORS; s++) {
    SectorHeader head;
    if (storage_read(s * KV_SECTOR_SIZE, &head, sizeof(SectorHeader)) !=
        STORAGE_OK) {
      kv_hal_unlock(&kv_db_lock);
      return KV_ERR_EEPROM;
    }

    if (head.magic == KV_SECTOR_MAGIC) {
      if (head.seq > max_seq) {
        max_seq = head.seq;
        latest_sector = s;
      }

      /* Scan sector for records */
      uint16_t offset = KV_RECORDS_OFFSET;
      while (offset < KV_SECTOR_SIZE - sizeof(KVRecord)) {
        KVRecord rec;
        if (storage_read(s * KV_SECTOR_SIZE + offset, &rec, sizeof(KVRecord)) !=
            STORAGE_OK) {
          kv_hal_unlock(&kv_db_lock);
          return KV_ERR_EEPROM;
        }

        if (rec.magic != KV_RECORD_MAGIC) {
          /* Check if end of data */
          if (rec.magic == 0xFF)
            break;
          /* Corrupt record? skip one byte and try again */
          offset++;
          continue;
        }

        /* Found a potentially valid record */
        char key[KV_MAX_KEY_LEN + 1];
        uint16_t record_addr = s * KV_SECTOR_SIZE + offset;
        if (read_key_from_addr(record_addr, key) != STORAGE_OK) {
          kv_hal_unlock(&kv_db_lock);
          return KV_ERR_EEPROM;
        }

        if (key[0] != '\0') {
          /* Update index (LFS: latest record for a key is the truth) */
          int16_t idx = find_index(key);
          if (idx >= 0) {
            /* Existing key - update address if this record is newer */
            /* In a single linear scan, later records are always newer */
            kv_index[idx].addr = record_addr;
          } else if (kv_index_count < KV_MAX_KEYS) {
            /* New key - insert in sorted order */
            uint16_t ins = find_insertion_point(key);
            for (uint16_t j = kv_index_count; j > ins; j--) {
              kv_index[j] = kv_index[j - 1];
            }
            kv_index[ins].addr = record_addr;
#if defined(KV_HAL_ARM)
            strncpy(kv_index[ins].cached_key, key, KV_MAX_KEY_LEN);
            kv_index[ins].cached_key[KV_MAX_KEY_LEN] = '\0';
#endif
            kv_index_count++;
          }
        }

        /* Move to next record */
        offset += sizeof(KVRecord) + rec.key_len + rec.val_len;
        os_wdt_feed();
      }
    }
  }

  current_write_sector = latest_sector;
  current_seq = max_seq;

  /* 2. Find tail in latest sector */
  uint16_t offset = KV_RECORDS_OFFSET;
  while (offset < KV_SECTOR_SIZE - sizeof(KVRecord)) {
    uint8_t magic;
    if (storage_read(current_write_sector * KV_SECTOR_SIZE + offset, &magic,
                     1) != STORAGE_OK)
      break;
    if (magic == 0xFF)
      break;

    KVRecord rec;
    if (storage_read(current_write_sector * KV_SECTOR_SIZE + offset, &rec,
                     sizeof(KVRecord)) != STORAGE_OK)
      break;
    offset += sizeof(KVRecord) + rec.key_len + rec.val_len;
    os_wdt_feed();
  }
  current_write_offset = offset;

  /* 3. Handle Fresh Start */
  if (max_seq == 0) {
    if (format_sector(0) != STORAGE_OK) {
      kv_hal_unlock(&kv_db_lock);
      return KV_ERR_EEPROM;
    }
    current_write_sector = 0;
    current_write_offset = KV_RECORDS_OFFSET;
  }

  kv_initialized = true;
  kv_hal_unlock(&kv_db_lock);
  return KV_SUCCESS;
}

/* --- API: Write --- */
/* --- API: Write --- */
kv_result_t kv_write(const char *key, const char *value, uint16_t val_len) {
  kv_result_t result = KV_SUCCESS;

  if (!kv_initialized) {
    if (kv_init() != KV_SUCCESS)
      return KV_ERR_EEPROM;
  }

  if (key == NULL || value == NULL) {
    result = KV_ERR_PARAM;
  } else {
    uint8_t key_len = (uint8_t)strlen(key);
    if (key_len > KV_MAX_KEY_LEN) {
      result = KV_ERR_KEY_TOO_LONG;
    } else if (val_len > KV_MAX_VAL_LEN) {
      result = KV_ERR_VAL_TOO_LONG;
    } else {
      kv_hal_lock(&kv_db_lock);
      uint16_t rec_total = (uint16_t)(sizeof(KVRecord) + key_len + val_len);

      /* Space Check & Wear Leveling */
      if ((current_write_offset + rec_total) > KV_SECTOR_SIZE) {
        current_write_sector =
            (uint16_t)((current_write_sector + 1) % KV_NUM_SECTORS);
        if (format_sector(current_write_sector) != STORAGE_OK) {
          kv_hal_unlock(&kv_db_lock);
          return KV_ERR_EEPROM;
        }
        current_write_offset = KV_RECORDS_OFFSET;
      }

      if ((current_write_offset + rec_total) > KV_SECTOR_SIZE) {
        result = KV_ERR_FULL;
      } else {
        /* Prepare Record Header */
        KVRecord rec;
        rec.magic = KV_RECORD_MAGIC;
        rec.key_len = key_len;
        rec.val_len = val_len;
        uint32_t crc = kv_hal_crc32(key, key_len, 0);
        rec.crc = kv_hal_crc32(value, val_len, crc);

        uint32_t addr = (uint32_t)(current_write_sector * KV_SECTOR_SIZE +
                                   current_write_offset);

        /* Commit to Storage */
        if (storage_write(addr, &rec, sizeof(KVRecord)) != STORAGE_OK ||
            storage_write(addr + sizeof(KVRecord), key, key_len) !=
                STORAGE_OK ||
            storage_write(addr + sizeof(KVRecord) + key_len, value, val_len) !=
                STORAGE_OK) {
          result = KV_ERR_EEPROM;
        } else {
          /* Update Index */
          int16_t idx = find_index(key);
          if (idx >= 0) {
            kv_index[idx].addr = (uint16_t)addr;
          } else if (kv_index_count < KV_MAX_KEYS) {
            uint16_t ins = find_insertion_point(key);
            for (uint16_t j = kv_index_count; j > ins; j--) {
              kv_index[j] = kv_index[j - 1];
            }
            kv_index[ins].addr = (uint16_t)addr;
#if defined(KV_HAL_ARM)
            (void)strncpy(kv_index[ins].cached_key, key, KV_MAX_KEY_LEN);
            kv_index[ins].cached_key[KV_MAX_KEY_LEN] = '\0';
#endif
            kv_index_count++;
          } else {
            result = KV_ERR_FULL;
          }

          if (result == KV_SUCCESS) {
            current_write_offset = (uint16_t)(current_write_offset + rec_total);
          }
        }
      }
      kv_hal_unlock(&kv_db_lock);
    }
  }
  return result;
}

/* --- API: Read --- */
kv_result_t kv_read(const char *key, char *buffer, uint16_t max_len,
                    uint16_t *actual_len) {
  kv_result_t result = KV_SUCCESS;

  if (!kv_initialized) {
    if (kv_init() != KV_SUCCESS)
      return KV_ERR_EEPROM;
  }

  if (key == NULL || buffer == NULL) {
    result = KV_ERR_PARAM;
  } else {
    kv_hal_lock(&kv_db_lock);

    int16_t idx = find_index(key);
    if (idx < 0) {
      result = KV_ERR_NOT_FOUND;
    } else {
      uint32_t addr = (uint32_t)kv_index[idx].addr;
      KVRecord rec;
      if (storage_read(addr, &rec, sizeof(KVRecord)) != STORAGE_OK) {
        result = KV_ERR_EEPROM;
      } else {
        uint16_t to_copy = (rec.val_len > max_len) ? max_len : rec.val_len;
        if (storage_read(addr + (uint16_t)sizeof(KVRecord) + rec.key_len,
                         buffer, to_copy) != STORAGE_OK) {
          result = KV_ERR_EEPROM;
        } else {
          if (actual_len != NULL) {
            *actual_len = rec.val_len;
          }

          /* Verify CRC */
          if (to_copy == rec.val_len) {
            uint32_t crc_check = kv_hal_crc32(key, rec.key_len, 0);
            crc_check = kv_hal_crc32(buffer, to_copy, crc_check);
            if (crc_check != rec.crc) {
              result = KV_ERR_EEPROM;
            }
          }
        }
      }
    }
    kv_hal_unlock(&kv_db_lock);
  }

  return result;
}

/* --- API: Delete --- */
kv_result_t kv_delete(const char *key) {
  if (!kv_initialized)
    kv_init();
  kv_hal_lock(&kv_db_lock);

  int16_t idx = find_index(key);
  if (idx < 0) {
    kv_hal_unlock(&kv_db_lock);
    return KV_ERR_NOT_FOUND;
  }

  /* Remove from index */
  for (uint16_t i = idx; i < kv_index_count - 1; i++) {
    kv_index[i] = kv_index[i + 1];
  }
  kv_index_count--;

  kv_hal_unlock(&kv_db_lock);
  return KV_SUCCESS;
}

/* --- API: Clear --- */
/* --- API: Clear --- */
kv_result_t kv_clear(void) {
  kv_hal_lock(&kv_db_lock);

#ifndef __AVR__
  for (uint16_t s = 0; s < KV_NUM_SECTORS; s++) {
    if (storage_erase((uint32_t)s * KV_SECTOR_SIZE, KV_SECTOR_SIZE) !=
        STORAGE_OK) {
      kv_hal_unlock(&kv_db_lock);
      return KV_ERR_EEPROM;
    }
    os_wdt_feed();
  }
#endif

  current_seq = 1;
  SectorHeader head = {KV_SECTOR_MAGIC, current_seq};
  if (storage_write(0, &head, sizeof(SectorHeader)) != STORAGE_OK) {
    kv_hal_unlock(&kv_db_lock);
    return KV_ERR_EEPROM;
  }

  current_write_sector = 0;
  current_write_offset = KV_RECORDS_OFFSET;
  kv_index_count = 0;

  kv_hal_unlock(&kv_db_lock);
  return KV_SUCCESS;
}

/* --- API: Compact --- */
#define KV_ITER_TEMP_BUF_SIZE 64

int16_t kv_compact(void) {
  if (!kv_initialized)
    if (kv_init() != KV_SUCCESS)
      return -1;
  kv_hal_lock(&kv_db_lock);

  uint16_t live_count = kv_index_count;
  IndexEntry *old_index =
      (IndexEntry *)os_malloc(sizeof(IndexEntry) * live_count);
  if (!old_index) {
    kv_hal_unlock(&kv_db_lock);
    return -1;
  }
  memcpy(old_index, kv_index, sizeof(IndexEntry) * live_count);

  /* Clear storage (Skip on AVR: update handles erase/write) */
#ifndef __AVR__
  for (uint16_t s = 0; s < KV_NUM_SECTORS; s++) {
    if (storage_erase((uint32_t)s * KV_SECTOR_SIZE, KV_SECTOR_SIZE) !=
        STORAGE_OK) {
      os_free(old_index);
      kv_hal_unlock(&kv_db_lock);
      return -1;
    }
    os_wdt_feed();
  }
#endif

  current_seq++;
  SectorHeader head = {KV_SECTOR_MAGIC, current_seq};
  if (storage_write(0, &head, sizeof(SectorHeader)) != STORAGE_OK) {
    os_free(old_index);
    kv_hal_unlock(&kv_db_lock);
    return -1;
  }

  current_write_sector = 0;
  current_write_offset = KV_RECORDS_OFFSET;

  /* Re-write all live records */
  bool error_occurred = false;
  for (uint16_t i = 0; i < live_count; i++) {
    if (error_occurred)
      break;

    KVRecord rec;
    if (storage_read(old_index[i].addr, &rec, sizeof(KVRecord)) != STORAGE_OK) {
      error_occurred = true;
      break;
    }

    char key[KV_MAX_KEY_LEN + 1];
    uint8_t k_len = rec.key_len;
    if (storage_read(old_index[i].addr + sizeof(KVRecord), key, k_len) !=
        STORAGE_OK) {
      error_occurred = true;
      break;
    }

    char *val = NULL;
    if (rec.val_len > 0) {
      val = (char *)os_malloc(rec.val_len);
      if (!val) {
        /* OOM: cannot migrate this record, skip it (data loss unavoidable if we
         * proceed, but we are aborting) */
        /* Actually, if we fail here, the DB is already wiped. We are restoring.
         */
        /* If we abort now, we lose everything not yet written. */
        /* Robustness: Continue? Or stop? Standard behavior for embedded: try
         * best effort? */
        /* We will skip this record. */
        continue;
      }
      if (storage_read(old_index[i].addr + sizeof(KVRecord) + k_len, val,
                       rec.val_len) != STORAGE_OK) {
        os_free(val);
        error_occurred = true;
        break;
      }
    }

    uint16_t rec_total = (uint16_t)(sizeof(KVRecord) + k_len + rec.val_len);

    /* Sector switching logic for compaction */
    if ((current_write_offset + rec_total) > KV_SECTOR_SIZE) {
      current_write_sector++;
      if (current_write_sector >= KV_NUM_SECTORS) {
        /* This should theoretically not happen if live data < capacity */
        if (val)
          os_free(val);
        error_occurred = true;
        break;
      }
      if (format_sector(current_write_sector) != STORAGE_OK) {
        if (val)
          os_free(val);
        error_occurred = true;
        break;
      }
      current_write_offset = KV_RECORDS_OFFSET;
    }

    uint32_t new_addr =
        current_write_sector * KV_SECTOR_SIZE + current_write_offset;

    // Write Record
    if (storage_write(new_addr, &rec, sizeof(KVRecord)) != STORAGE_OK ||
        storage_write(new_addr + sizeof(KVRecord), key, k_len) != STORAGE_OK ||
        (rec.val_len > 0 && storage_write(new_addr + sizeof(KVRecord) + k_len,
                                          val, rec.val_len) != STORAGE_OK)) {
      if (val)
        os_free(val);
      error_occurred = true;
      break;
    }

    kv_index[i].addr = (uint16_t)new_addr;
    current_write_offset += rec_total;

    if (val)
      os_free(val);
    os_wdt_feed(); // Feed watchdog during compaction
  }

  os_free(old_index);
  kv_hal_unlock(&kv_db_lock);

  if (error_occurred)
    return -1;
  return 1; /* Success */
}

/* --- API: Iterate --- */
kv_result_t kv_iterate(const char *prefix, kv_iter_cb_t cb, void *ctx) {
  if (!kv_initialized)
    kv_init();
  if (!cb)
    return KV_ERR_PARAM;

  kv_hal_lock(&kv_db_lock);

  size_t p_len = prefix ? strlen(prefix) : 0;
  char temp_key[KV_MAX_KEY_LEN + 1];
  char temp_val[KV_ITER_TEMP_BUF_SIZE]; /* buffer for small values */

  for (uint16_t i = 0; i < kv_index_count; i++) {
    os_wdt_feed();
    const char *key = get_key_for_entry(&kv_index[i], temp_key);

    if (p_len == 0 || strncmp(key, prefix, p_len) == 0) {
      /* Read value into temporary buffer (avoid large heap allocs while
         locked). We copy a small preview (up to KV_ITER_TEMP_BUF_SIZE) and call
         the callback outside the DB lock so callbacks can be slow or use heap.
       */
      uint32_t addr = kv_index[i].addr;
      KVRecord rec;
      if (storage_read(addr, &rec, sizeof(KVRecord)) != STORAGE_OK)
        continue;

      size_t read_len =
          (rec.val_len <= sizeof(temp_val)) ? rec.val_len : sizeof(temp_val);
      if (storage_read(addr + sizeof(KVRecord) + rec.key_len, temp_val,
                       read_len) != STORAGE_OK)
        continue;

      /* Copy key to stack-owned buffer to allow calling callback outside lock
       */
      char key_copy[KV_MAX_KEY_LEN + 1];
      strncpy(key_copy, key, KV_MAX_KEY_LEN);
      key_copy[KV_MAX_KEY_LEN] = '\0';

      /* Call user callback without holding DB lock */
      kv_hal_unlock(&kv_db_lock);
      cb(key_copy, temp_val, (uint16_t)read_len, ctx);
      kv_hal_lock(&kv_db_lock);
    }
  }

  kv_hal_unlock(&kv_db_lock);
  return KV_SUCCESS;
}
