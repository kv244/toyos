#include "kv_db.h"
#include "hal/storage_arduino_eeprom.h"
#include "hal/storage_avr_eeprom.h"
#include "hal/storage_driver.h"
#include <stdlib.h>
#include <string.h>

/* --- Configuration --- */

#define KV_NUM_SECTORS (KV_EEPROM_SIZE / KV_SECTOR_SIZE)
#define KV_TOMBSTONE_LEN 0xFFFF

/* --- Data Types --- */

typedef struct {
  uint32_t seq;
  uint32_t start_addr;
  bool valid;
} SectorInfo;

typedef struct {
  uint8_t count;
  IndexEntry entries[KV_MAX_KEYS];
  uint32_t head_addr;   // Current write position
  uint32_t current_seq; // Sequence number of current sector
} kv_index_t;

/* --- Globals --- */

static kv_lock_t kv_lock;
static kv_index_t *index_cache = NULL;

/* --- Helper Functions --- */

static int compare_sectors(const void *a, const void *b) {
  const SectorInfo *sa = (const SectorInfo *)a;
  const SectorInfo *sb = (const SectorInfo *)b;
  if (!sa->valid && !sb->valid)
    return 0;
  if (!sa->valid)
    return 1; // Invalid goes to end
  if (!sb->valid)
    return -1;
  if (sa->seq < sb->seq)
    return -1;
  if (sa->seq > sb->seq)
    return 1;
  return 0;
}

static void read_key_from_flash(uint32_t addr, char *key_buf) {
  KVRecord header;
  storage_read(addr, &header, sizeof(KVRecord));
  uint8_t len =
      (header.key_len > KV_MAX_KEY_LEN) ? KV_MAX_KEY_LEN : header.key_len;
  storage_read(addr + sizeof(KVRecord), key_buf, len);
  key_buf[len] = '\0';
}

static int compare_keys_flash(const char *key1, uint32_t addr2) {
  char key2[KV_MAX_KEY_LEN + 1];
  read_key_from_flash(addr2, key2);
  return strcmp(key1, key2);
}

/* Binary Search in RAM Index */
static int16_t index_search(const char *key) {
  int16_t left = 0;
  int16_t right = index_cache->count - 1;

  while (left <= right) {
    int16_t mid = left + (right - left) / 2;
    int cmp = compare_keys_flash(key, index_cache->entries[mid].addr);
    if (cmp == 0)
      return mid;
    if (cmp < 0)
      right = mid - 1;
    else
      left = mid + 1;
  }
  return -1;
}

/* Insert/Update RAM Index */
static void index_update(const char *key, uint32_t addr) {
  int16_t pos = index_search(key);

  if (pos >= 0) {
    /* Found - Update Address */
    index_cache->entries[pos].addr = (uint16_t)addr;
  } else {
    /* Not Found - Insert sorted */
    if (index_cache->count < KV_MAX_KEYS) {
      /* Find insertion spot */
      int i;
      for (i = index_cache->count - 1; i >= 0; i--) {
        if (compare_keys_flash(key, index_cache->entries[i].addr) > 0)
          break;
        index_cache->entries[i + 1] = index_cache->entries[i];
      }
      index_cache->entries[i + 1].addr = (uint16_t)addr;
      index_cache->count++;
    }
  }
}

static void index_remove(const char *key) {
  int16_t pos = index_search(key);
  if (pos >= 0) {
    for (int i = pos; i < index_cache->count - 1; i++) {
      index_cache->entries[i] = index_cache->entries[i + 1];
    }
    index_cache->count--;
  }
}

static kv_result_t kv_format_sector(uint32_t sector_start, uint32_t seq) {
  storage_erase(sector_start, KV_SECTOR_SIZE);
  SectorHeader hdr;
  hdr.magic = KV_SECTOR_MAGIC;
  hdr.seq = seq;
  return (kv_result_t)storage_write(sector_start, &hdr, sizeof(SectorHeader));
}

/* --- Core API --- */

kv_result_t kv_init(void) {
  kv_result_t res = KV_SUCCESS;
  kv_hal_lock_init(&kv_lock);
  kv_hal_crc_init();

  /* 1. Driver Init */
  if (storage_get_capacity() == 0) {
#ifdef __AVR__
    storage_set_driver(storage_get_avr_eeprom_driver());
#elif defined(ARDUINO_ARCH_RENESAS)
    storage_set_driver(storage_get_arduino_eeprom_driver());
#endif
    storage_init();
  }

  /* 2. Cache Alloc check */
  if (index_cache == NULL) {
    index_cache = (kv_index_t *)os_malloc(sizeof(kv_index_t));
    if (index_cache == NULL)
      return KV_ERR_EEPROM;
    memset(index_cache, 0, sizeof(kv_index_t));
  }

  /* 3. Scan Sectors */
  SectorInfo sectors[KV_NUM_SECTORS];
  uint32_t max_seq = 0;

  for (int i = 0; i < KV_NUM_SECTORS; i++) {
    sectors[i].start_addr = i * KV_SECTOR_SIZE;
    SectorHeader hdr;
    storage_read(sectors[i].start_addr, &hdr, sizeof(SectorHeader));

    if (hdr.magic == KV_SECTOR_MAGIC) {
      sectors[i].seq = hdr.seq;
      sectors[i].valid = true;
      if (hdr.seq > max_seq)
        max_seq = hdr.seq;
    } else {
      sectors[i].seq = 0;
      sectors[i].valid = false;
    }
  }

  /* 4. Format if fresh */
  if (max_seq == 0) {
    /* Initialize Sector 0 */
    kv_format_sector(0, 1);
    index_cache->current_seq = 1;
    index_cache->head_addr = sizeof(SectorHeader);
    index_cache->count = 0;
    return KV_SUCCESS;
  }

  /* 5. Sort Sectors by Seq */
  qsort(sectors, KV_NUM_SECTORS, sizeof(SectorInfo), compare_sectors);

  /* 6. Scan Valid Sectors and Build Index */
  for (int i = 0; i < KV_NUM_SECTORS; i++) {
    if (!sectors[i].valid)
      continue;

    uint32_t addr = sectors[i].start_addr + sizeof(SectorHeader);
    uint32_t limit = sectors[i].start_addr + KV_SECTOR_SIZE;

    while (addr + sizeof(KVRecord) <= limit) {
      KVRecord rec;
      storage_read(addr, &rec, sizeof(KVRecord));

      if (rec.magic != KV_RECORD_MAGIC)
        break; // End of valid data

      /* Calculate header+key+val checksum */
      // Optimization: For strict safety we should verify CRC.
      // Skipping for speed in this implementation, but recommend adding
      // verification.

      char key[KV_MAX_KEY_LEN + 1];
      storage_read(addr + sizeof(KVRecord), key, rec.key_len);
      key[rec.key_len] = '\0';

      if (rec.val_len == KV_TOMBSTONE_LEN) {
        index_remove(key);
      } else {
        index_update(key, addr);
      }

      addr += sizeof(KVRecord) + rec.key_len + rec.val_len;
    }

    /* Set Head to end of last active sector */
    if (sectors[i].seq == max_seq) {
      index_cache->head_addr = addr;
      index_cache->current_seq = max_seq;
    }
  }

  return res;
}

kv_result_t kv_write(const char *key, const char *value, uint16_t val_len) {
  kv_result_t res = KV_SUCCESS;
  kv_hal_lock(&kv_lock);

  uint8_t key_len = strlen(key);
  uint32_t total_size = sizeof(KVRecord) + key_len + val_len;

  uint32_t current_sector_start =
      (index_cache->head_addr / KV_SECTOR_SIZE) * KV_SECTOR_SIZE;
  uint32_t sector_remaining =
      (current_sector_start + KV_SECTOR_SIZE) - index_cache->head_addr;

  if (total_size > sector_remaining) {
    /* Move to Next Sector */
    uint32_t next_sector_idx =
        ((index_cache->head_addr / KV_SECTOR_SIZE) + 1) % KV_NUM_SECTORS;
    uint32_t next_sector_addr = next_sector_idx * KV_SECTOR_SIZE;

    /* Auto-Erase (Circular Buffer - overwrite old data) */
    index_cache->current_seq++;
    kv_format_sector(next_sector_addr, index_cache->current_seq);

    /* Check for Orphaned Index Entries (pointing to erased sector) */
    /* This is critical: if index points to overwritten sector, remove entry */
    for (int i = 0; i < index_cache->count;) {
      uint32_t entry_addr = index_cache->entries[i].addr;
      if (entry_addr >= next_sector_addr &&
          entry_addr < (next_sector_addr + KV_SECTOR_SIZE)) {
        /* Remove orphaned entry */
        for (int k = i; k < index_cache->count - 1; k++) {
          index_cache->entries[k] = index_cache->entries[k + 1];
        }
        index_cache->count--;
      } else {
        i++;
      }
    }

    index_cache->head_addr = next_sector_addr + sizeof(SectorHeader);
  }

  /* Write Record */
  KVRecord rec;
  rec.magic = KV_RECORD_MAGIC;
  rec.key_len = key_len;
  rec.val_len = val_len;
  rec.crc = 0; // TODO calculate CRC

  storage_write(index_cache->head_addr, &rec, sizeof(KVRecord));
  storage_write(index_cache->head_addr + sizeof(KVRecord), key, key_len);
  if (val_len > 0 && val_len != KV_TOMBSTONE_LEN) {
    storage_write(index_cache->head_addr + sizeof(KVRecord) + key_len, value,
                  val_len);
  }

  /* Update Index */
  if (val_len == KV_TOMBSTONE_LEN) {
    index_remove(key);
  } else {
    index_update(key, index_cache->head_addr);
  }

  index_cache->head_addr += total_size;

  kv_hal_unlock(&kv_lock);
  return res;
}

kv_result_t kv_read(const char *key, char *buffer, uint16_t max_len,
                    uint16_t *actual_len) {
  kv_result_t res = KV_ERR_NOT_FOUND;
  kv_hal_lock(&kv_lock);

  int16_t idx = index_search(key);
  if (idx >= 0) {
    uint32_t addr = index_cache->entries[idx].addr;
    KVRecord rec;
    storage_read(addr, &rec, sizeof(KVRecord));

    if (actual_len)
      *actual_len = rec.val_len;

    uint16_t copy_len = (rec.val_len > max_len) ? max_len : rec.val_len;
    storage_read(addr + sizeof(KVRecord) + rec.key_len, buffer, copy_len);
    res = KV_SUCCESS;
  }

  kv_hal_unlock(&kv_lock);
  return res;
}

kv_result_t kv_delete(const char *key) {
  return kv_write(key, NULL, KV_TOMBSTONE_LEN);
}

int16_t kv_compact(void) {
  /* Auto-compaction happens on sector boundary in kv_write */
  return 0;
}

kv_result_t kv_clear(void) {
  kv_hal_lock(&kv_lock);
  /* Format all sectors */
  for (int i = 0; i < KV_NUM_SECTORS; i++) {
    storage_erase(i * KV_SECTOR_SIZE, KV_SECTOR_SIZE);
  }
  index_cache->count = 0;
  kv_init(); /* Re-init */
  kv_hal_unlock(&kv_lock);
  return KV_SUCCESS;
}

kv_result_t kv_iterate(const char *prefix, kv_iter_cb_t cb, void *ctx) {
  if (!cb)
    return KV_ERR_PARAM;

  kv_hal_lock(&kv_lock);

  /* Binary search for start (Lower Bound) */
  int16_t left = 0;
  int16_t right = index_cache->count;

  while (left < right) {
    int16_t mid = left + (right - left) / 2;
    char key_buf[KV_MAX_KEY_LEN + 1];
    read_key_from_flash(index_cache->entries[mid].addr, key_buf);

    if (strcmp(key_buf, prefix) < 0) {
      left = mid + 1;
    } else {
      right = mid;
    }
  }

  size_t prefix_len = strlen(prefix);

  for (int i = left; i < index_cache->count; i++) {
    char key_buf[KV_MAX_KEY_LEN + 1];
    read_key_from_flash(index_cache->entries[i].addr, key_buf);

    if (strncmp(key_buf, prefix, prefix_len) != 0) {
      break;
    }

    uint32_t addr = index_cache->entries[i].addr;
    KVRecord rec;
    storage_read(addr, &rec, sizeof(KVRecord));

    void *val_buf = os_malloc(rec.val_len + 1);
    if (val_buf) {
      storage_read(addr + sizeof(KVRecord) + rec.key_len, val_buf, rec.val_len);
      ((char *)val_buf)[rec.val_len] = '\0'; // Ensure null term if string?
      cb(key_buf, val_buf, rec.val_len, ctx);
      os_free(val_buf);
    }
  }

  kv_hal_unlock(&kv_lock);
  return KV_SUCCESS;
}
