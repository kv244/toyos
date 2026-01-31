#ifndef KV_DB_H
#define KV_DB_H

#include <stdbool.h>
#include <stdint.h>
#include <toyos.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KV_MAX_KEY_LEN 24
#define KV_MAX_VAL_LEN 1024
#define KV_EEPROM_SIZE 1024

// Maximum number of keys (adjustable based on memory constraints)
#define KV_MAX_KEYS 18

// EEPROM layout offsets
#define KV_HEADER_OFFSET 0
#define KV_INDEX_OFFSET 4
#define KV_RECORDS_OFFSET 40

/**
 * EEPROM Header (4 bytes)
 * Stored at offset 0
 */
typedef struct {
  uint16_t magic;   // 0x4B56 ('KV')
  uint8_t count;    // Number of keys in index
  uint8_t reserved; // Reserved for future use
} KVHeader;

/**
 * Index entry stored in EEPROM.
 * The index is a sorted array of EEPROM addresses.
 */
typedef struct {
  uint16_t addr; // EEPROM address of the KV record
} IndexEntry;

/**
 * KV Record header stored in EEPROM.
 * Followed by key data and value data.
 */
typedef struct {
  uint8_t key_len;  // Length of key string (not including null)
  uint16_t val_len; // Length of value
  uint8_t flags;    // bit 0: is_free
} KVRecord;

#define KV_MAGIC 0x4B56
#define KV_FLAG_FREE 0x01

/**
 * Result codes for KV operations
 */
typedef enum {
  KV_SUCCESS = 0,
  KV_ERR_NOT_FOUND,
  KV_ERR_FULL,
  KV_ERR_KEY_TOO_LONG,
  KV_ERR_VAL_TOO_LONG,
  KV_ERR_EEPROM
} kv_result_t;

/**
 * Initialize the KV database.
 * Loads the sorted index from EEPROM or creates a new one.
 */
kv_result_t kv_init(void);

/**
 * Write a key-value pair to the database.
 * If the key already exists, updates the index to point to a new record.
 * The old record is marked as free but space is not immediately reclaimed.
 */
kv_result_t kv_write(const char *key, const char *value, uint16_t val_len);

/**
 * Read the value associated with a key.
 * buffer must be at least val_len specified in kv_write (or KV_MAX_VAL_LEN).
 */
kv_result_t kv_read(const char *key, char *buffer, uint16_t max_len,
                    uint16_t *actual_len);

/**
 * Delete a key from the database.
 * Removes the entry from the sorted index.
 * The EEPROM space is not reclaimed until kv_clear() is called.
 */
kv_result_t kv_delete(const char *key);

/**
 * Utility to clear the entire database.
 */
kv_result_t kv_clear(void);

#ifdef __cplusplus
}
#endif

#endif // KV_DB_H
