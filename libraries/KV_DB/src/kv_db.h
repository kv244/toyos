#ifndef KV_DB_H
#define KV_DB_H

#include <stdbool.h>
#include <stdint.h>
#include <toyos.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "kv_hal.h"

#define KV_MAX_KEY_LEN 24
#define KV_MAX_VAL_LEN 1024

#ifdef KV_HAL_ARM
#define KV_MAX_KEYS 64
#define KV_EEPROM_SIZE 8192 // R4 Data Flash is 8KB
#else
#define KV_MAX_KEYS 18
#define KV_EEPROM_SIZE 1024 // R3 EEPROM is 1KB
#endif

// EEPROM layout offsets
#define KV_HEADER_OFFSET 0
#define KV_INDEX_OFFSET 4
#define KV_RECORDS_OFFSET                                                      \
  132 // Increased to accommodate more keys on ARM (4 + 64*2)

/**
 * EEPROM Header (4 bytes)
 * Stored at offset 0
 */
typedef struct KV_ALIGN {
  uint16_t magic;   // 0x4B56 ('KV')
  uint8_t count;    // Number of keys in index
  uint8_t reserved; // Reserved for future use
} KV_PACKED KVHeader;

/**
 * Index entry stored in EEPROM.
 * The index is a sorted array of EEPROM addresses.
 */
typedef struct KV_ALIGN {
  uint16_t addr; // EEPROM address of the KV record
} KV_PACKED IndexEntry;

/**
 * KV Record header stored in EEPROM.
 * Followed by key data and value data.
 */
typedef struct KV_ALIGN {
  uint8_t key_len;  // Length of key string (not including null)
  uint16_t val_len; // Length of value
  uint8_t flags;    // bit 0: is_free
  uint32_t crc;     // Hardware CRC32 for ARM, XOR sum for AVR
} KV_PACKED KVRecord;

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
 * @brief Initialize the Key-Value database.
 *
 * Loads the sorted index from persistent storage into RAM.
 * If no valid database is found, formats the storage area.
 *
 * @return kv_result_t KV_SUCCESS on success, error code otherwise
 */
kv_result_t kv_init(void);

/**
 * @brief Write a key-value pair to the database.
 *
 * If the key already exists, updates the index to point to a new record.
 * The old record is marked as free but space is not immediately reclaimed.
 *
 * @param key Null-terminated key string (max KV_MAX_KEY_LEN)
 * @param value Pointer to value data
 * @param val_len Length of value data in bytes
 * @return kv_result_t KV_SUCCESS or error code
 */
kv_result_t kv_write(const char *key, const char *value, uint16_t val_len);

/**
 * @brief Read the value associated with a key.
 *
 * @param key Null-terminated key string to search for
 * @param buffer Output buffer to store the value
 * @param max_len Size of the output buffer
 * @param actual_len [out] Pointer to store actual value length (optional, can
 * be NULL)
 * @return kv_result_t KV_SUCCESS or error code
 */
kv_result_t kv_read(const char *key, char *buffer, uint16_t max_len,
                    uint16_t *actual_len);

/**
 * @brief Delete a key from the database.
 *
 * Removes the entry from the sorted index.
 * The EEPROM space is not reclaimed until kv_compact() is called.
 *
 * @param key Null-terminated key string to delete
 * @return kv_result_t KV_SUCCESS or error code
 */
kv_result_t kv_delete(const char *key);

/**
 * @brief Compact the database to reclaim free space.
 *
 * Moves active records to contiguous memory and updates the index.
 * Only writes to EEPROM/Flash if moving is necessary to save write cycles.
 *
 * @return int16_t Number of bytes reclaimed, or negative error code
 */
int16_t kv_compact(void);

/**
 * @brief Clear the entire database.
 *
 * Resets the header and clears the index. Data records are not explicitly
 * erased but become inaccessible and will be overwritten.
 *
 * @return kv_result_t KV_SUCCESS or error code
 */
kv_result_t kv_clear(void);

#ifdef __cplusplus
}
#endif

#endif // KV_DB_H
