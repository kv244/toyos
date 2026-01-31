#ifndef KV_DB_H
#define KV_DB_H

#include <stdbool.h>
#include <stdint.h>
#include <toyos.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "kv_hal.h"

#define KV_MAX_KEY_LEN 48

typedef void (*kv_iter_cb_t)(const char *key, const void *val, uint16_t val_len,
                             void *ctx);
#define KV_MAX_VAL_LEN 1024

#ifdef KV_HAL_ARM
#define KV_MAX_KEYS 64
#define KV_EEPROM_SIZE 8192 // R4 Data Flash is 8KB
#define KV_SECTOR_SIZE 1024 // 1KB Sector (8 sectors total)
#else
#define KV_MAX_KEYS 18
#define KV_EEPROM_SIZE 1024 // R3 EEPROM is 1KB
#define KV_SECTOR_SIZE 256  // 256B Sector (4 sectors total)
#endif

// Sector Header (8 bytes)
// Stored at the beginning of each sector.
typedef struct KV_ALIGN {
  uint32_t magic; // 0xDEADBEEF
  uint32_t seq;   // Monotonic sequence number
} KV_PACKED SectorHeader;

#define KV_SECTOR_MAGIC 0xDEADBEEF

/**
 * KV Record Header (Fixed size)
 * Followed by key data and value data.
 */
typedef struct KV_ALIGN {
  uint8_t magic;    // 0xA5
  uint8_t key_len;  // Length of key string
  uint16_t val_len; // Length of value
  uint32_t crc;     // Checksum of Key + Value
} KV_PACKED KVRecord;

#define KV_RECORD_MAGIC 0xA5

/**
 * Index entry (RAM only).
 */
typedef struct KV_ALIGN {
  uint16_t addr; // Address of valid record
#if defined(KV_HAL_ARM)
  char cached_key[KV_MAX_KEY_LEN + 1];
#endif
} KV_PACKED IndexEntry;

/**
 * Result codes for KV operations
 */
typedef enum {
  KV_SUCCESS = 0,
  KV_ERR_NOT_FOUND,
  KV_ERR_FULL,
  KV_ERR_KEY_TOO_LONG,
  KV_ERR_VAL_TOO_LONG,
  KV_ERR_EEPROM,
  KV_ERR_PARAM
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

/**
 * @brief Iterate over keys starting with a prefix.
 *
 * @param prefix Null-terminated prefix string. "" matches all keys.
 * @param cb Callback function to invoke for each match.
 * @param ctx user context passed to callback.
 * @return kv_result_t
 */
kv_result_t kv_iterate(const char *prefix, kv_iter_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif

#endif // KV_DB_H
