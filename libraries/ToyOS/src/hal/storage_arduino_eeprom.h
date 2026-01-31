/**
 * Arduino EEPROM Storage Driver (Works on AVR and R4 via emulation)
 */

#ifndef TOYOS_STORAGE_ARDUINO_EEPROM_H
#define TOYOS_STORAGE_ARDUINO_EEPROM_H

#include "storage_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Get the singleton instance of the Arduino EEPROM driver */
const storage_driver_t *storage_get_arduino_eeprom_driver(void);

#ifdef __cplusplus
}
#endif

#endif
