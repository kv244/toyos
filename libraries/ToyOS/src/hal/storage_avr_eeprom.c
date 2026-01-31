/**
 * AVR EEPROM Storage Driver Implementation
 */

#include "storage_avr_eeprom.h"

#if defined(__AVR__)
#include <avr/eeprom.h>
#include <avr/io.h>

static storage_result_t avr_eeprom_init(void) {
  /* valid for AVR */
  return STORAGE_OK;
}

static storage_result_t avr_eeprom_read(uint32_t addr, void *buf, size_t len) {
  if (addr + len > E2END + 1) {
    return STORAGE_ERR_INVALID_ADDR;
  }
  eeprom_read_block(buf, (const void *)(uintptr_t)addr, len);
  return STORAGE_OK;
}

static storage_result_t avr_eeprom_write(uint32_t addr, const void *buf,
                                         size_t len) {
  if (addr + len > E2END + 1) {
    return STORAGE_ERR_INVALID_ADDR;
  }
  /* eeprom_update_block is better than write (saves cycles if unchanged) */
  eeprom_update_block(buf, (void *)(uintptr_t)addr, len);
  return STORAGE_OK;
}

static uint32_t avr_eeprom_capacity(void) { return E2END + 1; }

static const storage_driver_t avr_eeprom_driver = {
    .init = avr_eeprom_init,
    .read = avr_eeprom_read,
    .write = avr_eeprom_write,
    .erase = NULL, /* eeprom_update handles erase/write automatically */
    .get_capacity = avr_eeprom_capacity,
    .name = "AVR EEPROM"};

const storage_driver_t *storage_get_avr_eeprom_driver(void) {
  return &avr_eeprom_driver;
}

#else

/* Stub for non-AVR platforms (so it compiles) */
const storage_driver_t *storage_get_avr_eeprom_driver(void) { return NULL; }

#endif
