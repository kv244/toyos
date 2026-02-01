#include <Arduino.h>

#include <kv_db.h>
#include <toyos.h>

#ifdef __arm__
static uint8_t mem_pool[8192] __attribute__((aligned(8)));
#else
static uint8_t mem_pool[896] __attribute__((aligned(2)));
#endif

Mutex serial_mutex;

extern "C" void run_all_tests(void);

void log_msg(const char *msg) {
  os_mutex_lock(&serial_mutex);
  os_print(msg);
  os_mutex_unlock(&serial_mutex);
}

void log_f(const __FlashStringHelper *msg) {
  os_mutex_lock(&serial_mutex);
  Serial.println(msg);
  os_mutex_unlock(&serial_mutex);
}

void task_db_demo(void) {
  log_f(F("[DB] Initializing KV Database..."));

  if (kv_init() != KV_SUCCESS) {
    log_f(F("[DB] Error initializing database!"));
    while (1)
      os_delay(1000);
  }

  run_all_tests();
  log_f(F("[DB] Starting KV Database Demo..."));

  // 1. Write a key
  log_f(F("[DB] Writing key 'username' with value 'julia'..."));
  if (kv_write("username", "julia", 5) == KV_SUCCESS) {
    log_f(F("[DB] Write successful."));
  }

  // 2. Read the key back
  char buffer[32];
  uint16_t actual_len;
  log_f(F("[DB] Reading key 'username'..."));
  if (kv_read("username", buffer, sizeof(buffer), &actual_len) == KV_SUCCESS) {
    buffer[actual_len] = '\0';
    os_mutex_lock(&serial_mutex);
    Serial.print(F("[DB] Got value: "));
    Serial.println(buffer);
    os_mutex_unlock(&serial_mutex);
  }

  // 3. Update the key
  log_f(F("[DB] Updating 'username' to 'antigravity'..."));
  kv_write("username", "antigravity", 11);

  if (kv_read("username", buffer, sizeof(buffer), &actual_len) == KV_SUCCESS) {
    buffer[actual_len] = '\0';
    os_mutex_lock(&serial_mutex);
    Serial.print(F("[DB] New value: "));
    Serial.println(buffer);
    os_mutex_unlock(&serial_mutex);
  }

  // 4. Delete the key
  log_f(F("[DB] Deleting 'username'..."));
  kv_delete("username");

  if (kv_read("username", buffer, sizeof(buffer), &actual_len) ==
      KV_ERR_NOT_FOUND) {
    log_f(F("[DB] Read failed as expected: Key not found."));
  }

  // 5. Demonstrate compaction
  log_f(F("[DB] Running compaction to reclaim EEPROM space..."));
  int16_t reclaimed = kv_compact();
  if (reclaimed >= 0) {
    os_mutex_lock(&serial_mutex);
    Serial.print(F("[DB] Compaction successful. Reclaimed "));
    Serial.print(reclaimed);
    Serial.println(F(" bytes."));
    os_mutex_unlock(&serial_mutex);
  } else {
    log_f(F("[DB] Compaction failed!"));
  }

  log_f(F("[DB] Demo finished."));

  while (1) {
    os_delay(1000);
  }
}

void task_security_violation(void) {
  os_delay(2000);
#ifdef __arm__
  log_f(F("[SEC] MPU Test: Attempting to write to protected kernel memory..."));
  /* Try to write to the beginning of SRAM (Region 0 - Kernel) */
  volatile uint32_t *kernel_mem = (volatile uint32_t *)0x20000000;
  *kernel_mem = 0xBAD0CAFE;

  /* If we reach here, MPU failed! */
  log_f(F("[SEC] ERROR: MPU failed to block kernel write!"));
#endif
  while (1)
    os_delay(1000);
}

void task_idle(void) {
  while (1) {
    os_wdt_feed();
    os_check_stack_overflow();
    os_enter_idle();
  }
}

/**
 * Print detailed system information (Version, Platform, Memory).
 * This function uses Serial port and blocks until output is done.
 * Best called during setup() before os_start().
 */
void os_print_info(void);

/**
 * Print a string to the system console.
 * When MPU is enabled, this uses an SVC call to safely access the Serial
 * hardware.
 * @param msg Null-terminated string to print
 */
void os_print(const char *msg);

void setup() {
  /* Initialize serial for debug output */
  Serial.begin(9600);
  while (!Serial)
    ; /* Wait for serial connection */

  /* Print ToyOS Platform Info */
  os_print_info();

  Serial.println(F("\n--- ToyOS KV Database Demo ---"));

  /* Initialize memory pool for stack allocation */
  os_init(mem_pool, sizeof(mem_pool));

  /* Create synchronization primitives */
  os_mutex_init(&serial_mutex);

  /* Note: watchdog is now handled by port layer */

  /* Create tasks */
  os_create_task(1, task_db_demo, 1,
                 350); /* ID 1, DB Task, Prio 1, Stack 350 */
  os_create_task(2, task_idle, 0, 100);
  os_create_task(3, task_security_violation, 2, 128);

  /* Start the RTOS (never returns) */
  os_start();
}

void loop() {}
