/**
 * ToyOS Storage HAL Implementation
 */

#include "storage_driver.h"

#include "storage_arduino_eeprom.h"

/* Active driver pointer - defaults to NULL */
const storage_driver_t *current_storage_driver = NULL;

storage_result_t storage_bind_platform_driver(void) {
#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_UNOR4_MINIMA) ||          \
    defined(ARDUINO_UNOR4_WIFI)
  current_storage_driver = storage_get_arduino_eeprom_driver();
#else
  current_storage_driver = NULL;
  return STORAGE_ERR_INIT_FAILED;
#endif

  if (current_storage_driver == NULL) {
    return STORAGE_ERR_INIT_FAILED;
  }
  return STORAGE_OK;
}

void storage_set_driver(const storage_driver_t *driver) {
  current_storage_driver = driver;
}
