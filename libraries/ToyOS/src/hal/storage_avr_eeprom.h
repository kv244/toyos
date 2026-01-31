/**
 * AVR EEPROM Storage Driver
 */

#ifndef TOYOS_STORAGE_AVR_EEPROM_H
#define TOYOS_STORAGE_AVR_EEPROM_H

#include "storage_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Get the singleton instance of the AVR EEPROM driver */
const storage_driver_t *storage_get_avr_eeprom_driver(void);

#ifdef __cplusplus
}
#endif

#endif
