/**
 * ToyOS Storage HAL Implementation
 */

#include "storage_driver.h"

/* Active driver pointer - defaults to NULL */
const storage_driver_t *current_storage_driver = NULL;

void storage_set_driver(const storage_driver_t *driver) {
  current_storage_driver = driver;
}
