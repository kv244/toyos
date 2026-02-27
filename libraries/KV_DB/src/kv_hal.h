#ifndef KV_HAL_H
#define KV_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include <toyos.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Platform Detection & Alignment --- */

#if defined(__arm__) || defined(__thumb__)
/* ARM Cortex-M (Uno R4) */
#define KV_HAL_ARM
#define KV_ALIGN __attribute__((aligned(4)))
#define KV_PACKED __attribute__((packed))
#define KV_RAM_FUNC __attribute__((section(".ram_code")))
#define KV_MEMORY_BARRIER() __asm volatile("dmb sy" ::: "memory")
#else
/* AVR (Uno R3) */
#define KV_HAL_AVR
#define KV_ALIGN
#define KV_PACKED __attribute__((packed))
#define KV_RAM_FUNC
#define KV_MEMORY_BARRIER()
#endif

/* --- Lock Type & Abstraction --- */

#ifdef KV_HAL_ARM
typedef volatile uint32_t kv_lock_t;

static inline void kv_hal_lock_init(kv_lock_t *lock) { *lock = 0; }

static inline void kv_hal_lock(kv_lock_t *lock) {
  /* Optimization: Use LDREX/STREX for lock-free concurrency where possible,
     but for DB consistency, we often still want to block or yield.
     Since ToyOS has Mutex, we'll use a mix or just wrap Mutex.
     The user specifically suggested LDREX/STREX.
  */
  uint32_t status;
  do {
    __asm volatile("ldrex r1, [%1] \n"
                   "cmp r1, #0 \n"
                   "itt eq \n"
                   "strexeq r1, %2, [%1] \n"
                   "moveq %0, r1 \n"
                   "bne 1f \n"
                   "b 2f \n"
                   "1: mov %0, #1 \n"
                   "2: \n"
                   : "=&r"(status)
                   : "r"(lock), "r"(1)
                   : "r1", "memory");
    if (status != 0) {
      os_delay(1); // Yield if locked
    }
  } while (status != 0);
  KV_MEMORY_BARRIER();
}

static inline void kv_hal_unlock(kv_lock_t *lock) {
  KV_MEMORY_BARRIER();
  *lock = 0;
  KV_MEMORY_BARRIER();
}
#else
typedef Mutex kv_lock_t;

static inline void kv_hal_lock_init(kv_lock_t *lock) { os_mutex_init(lock); }

static inline void kv_hal_lock(kv_lock_t *lock) { os_mutex_lock(lock); }

static inline void kv_hal_unlock(kv_lock_t *lock) { os_mutex_unlock(lock); }
#endif

/* --- CRC32 Abstraction --- */

#ifdef KV_HAL_ARM
/* RA4M1 CRC Peripheral */
#define RA4M1_CRC_BASE 0x40074000
#define RA4M1_CRCCR (*(volatile uint8_t *)(RA4M1_CRC_BASE + 0x00))
#define RA4M1_CRCDIR (*(volatile uint8_t *)(RA4M1_CRC_BASE + 0x01))
#define RA4M1_CRCDOR (*(volatile uint32_t *)(RA4M1_CRC_BASE + 0x04))

#define RA4M1_MSTPCRC_ADDR 0x40047010 // MSTPCRA offset for CRC bit 19
#define RA4M1_MSTPCRC_BIT 19

static inline void kv_hal_crc_init(void) {
  /* RA4M1 PRCR (Protect Register) unlock: bit 1 (PRC1) for Low Power regs */
  volatile uint16_t *prcr = (volatile uint16_t *)0x4001E01E;
  *prcr = 0xA502; // Unlock MSTP regs

  /* Enable CRC clock: MSTPCRA bit 19 = 0 to enable */
  volatile uint32_t *mstpcra = (volatile uint32_t *)RA4M1_MSTPCRC_ADDR;
  *mstpcra &= ~(1UL << RA4M1_MSTPCRC_BIT);

  *prcr = 0xA500; // Lock back

  /* Configure CRC: 32-bit, Ethernet polynomial usually */
  /* CRCCR: bit 7: clear, bits 1-0: 11 for CRC-32 */
  RA4M1_CRCCR = 0x83;
}

static inline uint32_t kv_hal_crc32(const void *data, uint16_t len,
                                    uint32_t seed) {
  const uint8_t *p = (const uint8_t *)data;
  RA4M1_CRCDOR = seed; // Seed the CRC result
  for (uint16_t i = 0; i < len; i++) {
    RA4M1_CRCDIR = p[i];
  }
  return RA4M1_CRCDOR;
}
#else
static inline void kv_hal_crc_init(void) {}
static inline uint32_t kv_hal_crc32(const void *data, uint16_t len,
                                    uint32_t seed) {
  /* Simple software XOR sum as fallback for AVR */
  uint32_t crc = seed;
  const uint8_t *p = (const uint8_t *)data;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= p[i];
  }
  return crc;
}
#endif

/* --- Storage Mapping --- */

#ifdef KV_HAL_ARM
/* R4 Data Flash starts at 0x08000000 */
#define KV_STORAGE_BASE 0x08000000
#else
/* R3 EEPROM handled by driver address 0 */
#define KV_STORAGE_BASE 0
#endif

#ifdef __cplusplus
}
#endif

#endif /* KV_HAL_H */
