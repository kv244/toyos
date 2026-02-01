#include <Arduino.h>

#include <kv_db.h>
#include <toyos.h>

#ifndef TOYOS_DEBUG_ALLOC
#define TOYOS_DEBUG_ALLOC 0
#endif

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
/*
 * Memory adjustments and diagnostics (actions taken):
 * - Reduced mem_pool to 1024 bytes to avoid SRAM exhaustion on ATmega328P.
 * - Reduced task stacks (idle=64, db=256) to conserve heap for dynamic
 * allocations.
 * - Added early boot prints ("BOOT: early") and a MCUSR reset-reason report to
 */
static uint8_t mem_pool[800]
    __attribute__((aligned(2))); // Ultra-safe pool size
#endif

Mutex serial_mutex;

void log_msg(const char *msg) {
  Serial.println(msg);
  Serial.flush();
}

void log_f(const __FlashStringHelper *msg) {
  Serial.println(msg);
  Serial.flush();
}

/* --- CLI Helpers --- */
void show_keys_cb(const char *key, const void *val, uint16_t len, void *ctx) {
  os_mutex_lock(&serial_mutex);
  Serial.print(F("  "));
  Serial.print(key);
  Serial.print(F(": "));
  /* Print up to a short preview to avoid long blocking prints */
  const char *v = (const char *)val;
  uint16_t print_len = (len > 64) ? 64 : len;
  for (uint16_t i = 0; i < print_len; i++) {
    Serial.print(v[i]);
  }
  if (len > print_len) {
    Serial.print(F("..."));
  }
  Serial.println();
  os_mutex_unlock(&serial_mutex);
}

void process_command(char *cmd) {
  /* Normalize command to uppercase for checking (simple approach) */
  /* Actually, let's just check the prefix case-insensitively or strictly.
   * User asked for ADD/DELETE caps in prompt. */

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
  /* Initialize storage driver */
#ifdef __AVR__
  storage_set_driver(storage_get_avr_eeprom_driver());
#else
  storage_set_driver(storage_get_arduino_eeprom_driver());
#endif
  storage_init();

  /* Initialize KV Database */
  kv_result_t init_result = kv_init();

  os_mutex_lock(&serial_mutex);
  Serial.print(F("[DB] init result="));
  Serial.println((int)init_result);
  Serial.println(F("[DB] CLI Ready."));
  Serial.println(
      F("Commands: ADD K,V | UPDATE K,V | DELETE K | SHOW KEYS | CAPACITY"));
  Serial.print(F("> "));
  Serial.flush();
  os_mutex_unlock(&serial_mutex);

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
        os_mutex_lock(&serial_mutex);
        Serial.print(F("> "));
        Serial.flush();
        os_mutex_unlock(&serial_mutex);
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

/* Lightweight heartbeat task: prints a short line and toggles LED every 2s */
/* Auxiliary heartbeat and probe tasks removed to conserve RAM on UNO. */

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

  /* Blink built-in LED to indicate the board booted */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  /* Initialize serial for debug output (faster baud) */
  Serial.begin(115200);
  unsigned long tstart = millis();
  while (!Serial && (millis() - tstart) < 2000)
    ; /* Wait up to 2s for serial connection */

#if TOYOS_DEBUG_ENABLED
  /* Early serial sanity check */
  Serial.println(F("BOOT: early"));
  Serial.flush();
#endif

  /* Report reset cause (MCUSR) on AVR to detect watchdog/brown-out/etc */
#ifdef __AVR__
  {
    uint8_t mcusr_val = MCUSR;
#if TOYOS_DEBUG_ENABLED
    Serial.print(F("BOOT: MCUSR=0x"));
    Serial.println(mcusr_val, HEX);
#endif
    if (mcusr_val & (1 << WDRF))
      Serial.println(F("BOOT: Reset cause = WDT"));
    if (mcusr_val & (1 << BORF))
      Serial.println(F("BOOT: Reset cause = BOR"));
    if (mcusr_val & (1 << EXTRF))
      Serial.println(F("BOOT: Reset cause = EXTERNAL"));
    if (mcusr_val & (1 << PORF))
      Serial.println(F("BOOT: Reset cause = POR"));
    MCUSR = 0; /* Clear flags for subsequent boots */
    Serial.flush();

    /* Diagnostic: disable watchdog early to prevent WDT-induced resets while
       debugging. This is a temporary measure; do not rely on it in
       production.
     */
    wdt_disable();
#if TOYOS_DEBUG_ENABLED
    Serial.println(F("BOOT: WDT disabled (diagnostic)"));
    Serial.flush();
#endif
  }
#endif

  /* Emit a single boot trace and pulse LED once for visual confirmation */
#if TOYOS_DEBUG_ENABLED
  Serial.println(F("BOOT TRACE"));
  Serial.flush();
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(F("\n--- ToyOS KV Database Console ---"));
  Serial.flush();

  /* Initialize memory pool for stack allocation */
  os_init(mem_pool, sizeof(mem_pool));
#if TOYOS_DEBUG_ENABLED
  Serial.println(F("DBG: os_init called"));
  Serial.flush();
#endif

  /* Create tasks */
  os_create_task(2, task_idle, 0, 160);    // ID 2, Pri 0, Stack 160
  os_create_task(1, task_db_demo, 1, 384); // ID 1, Pri 1, Stack 384

#if TOYOS_DEBUG_ENABLED
  Serial.println(F("REACHED END OF SETUP"));
  Serial.flush();
#endif

  /* Start the RTOS (never returns) */
  os_start();
}

void loop() {}
