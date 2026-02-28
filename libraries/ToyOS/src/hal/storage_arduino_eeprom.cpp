/**
 * @file storage_arduino_eeprom.cpp
 * @brief Arduino EEPROM storage driver implementation.
 *
 * Provides an abstraction layer for the native Arduino EEPROM library,
 * specifically optimized for Renesas R4 platforms with watchdog feeding.
 */

#include "storage_arduino_eeprom.h"

#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_UNOR4_MINIMA) ||          \
    defined(ARDUINO_UNOR4_WIFI)
#include <Arduino.h>
#include <EEPROM.h>
#include <toyos.h>

static storage_result_t arduino_eeprom_init(void) { return STORAGE_OK; }

static storage_result_t arduino_eeprom_read(uint32_t addr, void *buf,
                                            size_t len) {
  uint8_t *p = (uint8_t *)buf;
  for (size_t i = 0; i < len; i++) {
    p[i] = EEPROM.read(addr + i);
    if ((i & 0x7F) == 0)
      os_wdt_feed();
  }
  return STORAGE_OK;
}

static storage_result_t arduino_eeprom_write(uint32_t addr, const void *buf,
                                             size_t len) {
  const uint8_t *p = (const uint8_t *)buf;
  for (size_t i = 0; i < len; i++) {
    /* update() writes only if changed, saving cycles/wear */
    EEPROM.update(addr + i, p[i]);
    if ((i & 0x3F) == 0)
      os_wdt_feed();
  }
  return STORAGE_OK;
}

static uint32_t arduino_eeprom_capacity(void) { return EEPROM.length(); }

static storage_result_t arduino_eeprom_erase(uint32_t addr, size_t len) {
  for (size_t i = 0; i < len; i++) {
    EEPROM.update(addr + i, 0xFF);
    if ((i & 0x3F) == 0)
      os_wdt_feed();
  }
  return STORAGE_OK;
}

static const storage_driver_t arduino_eeprom_driver = {
    .init = arduino_eeprom_init,
    .read = arduino_eeprom_read,
    .write = arduino_eeprom_write,
    .erase = arduino_eeprom_erase,
    .get_capacity = arduino_eeprom_capacity,
    .name = "Arduino EEPROM"};

const storage_driver_t *storage_get_arduino_eeprom_driver(void) {
  return &arduino_eeprom_driver;
}

#else

const storage_driver_t *storage_get_arduino_eeprom_driver(void) { return NULL; }

#endif
