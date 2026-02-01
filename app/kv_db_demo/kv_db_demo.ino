#include <Arduino.h>

#include <kv_db.h>
#include <toyos.h>

/* Include storage driver for AVR/ARM */
#ifdef __AVR__
#include <avr/wdt.h> // For watchdog control
#include <hal/storage_avr_eeprom.h>

#else
#include <hal/storage_arduino_eeprom.h>
#endif

#ifdef __arm__
static uint8_t mem_pool[8192] __attribute__((aligned(8)));
#else
static uint8_t mem_pool[640]
    __attribute__((aligned(2))); // Reduced from 896 to fit RAM
#endif

Mutex serial_mutex;

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

/* --- CLI Helpers --- */
void show_keys_cb(const char *key, void *val, uint16_t len, void *ctx) {
  os_mutex_lock(&serial_mutex);
  Serial.print(F("  "));
  Serial.print(key);
  Serial.print(F(": "));
  /* Value might not be null-terminated safe string, print char by char */
  const char *v = (const char *)val;
  for (uint16_t i = 0; i < len; i++) {
    Serial.print(v[i]);
  }
  Serial.println();
  os_mutex_unlock(&serial_mutex);
}

void process_command(char *cmd) {
  /* Normalize command to uppercase for checking (simple approach) */
  /* Actually, let's just check the prefix case-insensitively or strictly. User
   * asked for ADD/DELETE caps in prompt. */

  if (strncmp(cmd, "ADD ", 4) == 0 || strncmp(cmd, "UPDATE ", 7) == 0) {
    char *args = strchr(cmd, ' ');
    if (!args)
      return;
    args++; /* Skip space */

    char *comma = strchr(args, ',');
    if (!comma) {
      log_f(F("Error: Use format 'KEY, VALUE'"));
      return;
    }

    *comma = '\0'; /* Null-terminate key */
    char *val = comma + 1;
    while (*val == ' ')
      val++; /* Skip leading spaces in value */

    if (kv_write(args, val, strlen(val)) == KV_SUCCESS) {
      log_f(F("OK"));
    } else {
      log_f(F("Error: Write failed (Full/Too Long?)"));
    }

  } else if (strncmp(cmd, "DELETE ", 7) == 0) {
    char *key = cmd + 7;
    while (*key == ' ')
      key++;

    if (kv_delete(key) == KV_SUCCESS) {
      log_f(F("Deleted"));
    } else {
      log_f(F("Error: Key not found"));
    }

  } else if (strncmp(cmd, "SHOW KEYS", 9) == 0) {
    log_f(F("--- Keys ---"));
    kv_iterate(NULL, show_keys_cb, NULL);
    log_f(F("------------"));

  } else if (strncmp(cmd, "CAPACITY", 8) == 0) {
    /* Estimate Capacity for Key(20) + Value(100) */
    /* Overhead: Magic(1)+KeyLen(1)+ValLen(2)+CRC(4) = 8 bytes */
    /* Record Size = 8 + 20 + 100 = 128 bytes */
    /* Total Capacity = (EEPROM - SectorHeaders)/128 */
    /* Constraint: KV_MAX_KEYS (RAM Index) */

    uint32_t total_storage = KV_EEPROM_SIZE;
    uint32_t sector_header_overhead =
        (KV_EEPROM_SIZE / KV_SECTOR_SIZE) * sizeof(SectorHeader);
    uint32_t usable_bytes = total_storage - sector_header_overhead;

    uint16_t key_len = 20;
    uint16_t val_len = 100;
    uint16_t rec_size = sizeof(KVRecord) + key_len + val_len;

    uint32_t storage_max = usable_bytes / rec_size;
    uint32_t ram_max = KV_MAX_KEYS;

    uint32_t actual_max = (storage_max < ram_max) ? storage_max : ram_max;

    os_mutex_lock(&serial_mutex);
    Serial.print(F("Platform Capacity Estimate (K=20, V=100):\n"));
    Serial.print(F("  Storage Limit: "));
    Serial.println(storage_max);
    Serial.print(F("  RAM Limit:     "));
    Serial.println(ram_max);
    Serial.print(F("  Actual Max:    "));
    Serial.println(actual_max);
    os_mutex_unlock(&serial_mutex);

  } else if (strlen(cmd) > 0) {
    log_f(F("Unknown command. Try: ADD K,V | DELETE K | SHOW KEYS | CAPACITY"));
  }
}

void task_db_demo(void) {
  /* Give idle task time to start and feed watchdog */
  os_delay(100);

  Serial.println(F("[DB] Task started"));
  Serial.flush();

  /* Initialize storage driver */
  Serial.println(F("[DB] Initializing storage driver..."));
  Serial.flush();

#ifdef __AVR__
  storage_set_driver(storage_get_avr_eeprom_driver());
#else
  storage_set_driver(storage_get_arduino_eeprom_driver());
#endif

  if (storage_init() != STORAGE_OK) {
    Serial.println(F("[DB] Storage init failed!"));
    Serial.flush();
    while (1)
      os_delay(1000);
  }

  Serial.println(F("[DB] Storage driver ready"));
  Serial.flush();

  log_f(F("[DB] Initializing KV Database..."));
  Serial.flush();

  kv_result_t init_result = kv_init();

  Serial.print(F("[DB] Init result: "));
  Serial.println(init_result);
  Serial.flush();

  if (init_result != KV_SUCCESS) {
    log_f(F("[DB] Error initializing database!"));
    Serial.flush();
    while (1)
      os_delay(1000);
  }

  log_f(F("[DB] CLI Ready."));
  Serial.flush();
  log_f(F("Commands: ADD K,V | UPDATE K,V | DELETE K | SHOW KEYS | CAPACITY"));
  Serial.flush();
  Serial.print(F("> "));
  Serial.flush();

  static char line_buf[64];
  static uint8_t line_pos = 0;

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      /* Echo character */
      Serial.write(c);

      if (c == '\n' || c == '\r') {
        Serial.println(); /* Newline for visual */
        if (line_pos > 0) {
          line_buf[line_pos] = '\0';
          process_command(line_buf);
          line_pos = 0;
        }
        Serial.print(F("> "));
      } else if (line_pos < sizeof(line_buf) - 1) {
        if (c >= 32 && c <= 126) { /* printable only */
          line_buf[line_pos++] = c;
        }
      }
    }
    os_delay(20); /* Yield to other tasks */
  }
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
#ifdef __AVR__
  /* Disable watchdog immediately to prevent boot loop */
  wdt_disable();
#endif

  /* Initialize serial for debug output */
  Serial.begin(9600);
  while (!Serial)
    ; /* Wait for serial connection */

  /* Print ToyOS Platform Info */
  // os_print_info(); // Commented out to diagnose boot loop

  Serial.println(F("\n--- ToyOS KV Database Console ---"));

  /* Initialize memory pool for stack allocation */
  os_init(mem_pool, sizeof(mem_pool));

  /* Create synchronization primitives */
  os_mutex_init(&serial_mutex);

  /* Create tasks - idle has priority 2 to feed watchdog during init */
  os_create_task(2, task_idle, 2, 80);     /* Reduced stack: 100 -> 80 */
  os_create_task(1, task_db_demo, 1, 400); /* Reduced stack: 512 -> 400 */

  /* Start the RTOS (never returns) */
  os_start();
}

void loop() {}
