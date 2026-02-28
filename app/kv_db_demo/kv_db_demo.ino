/**
 * @file kv_db_demo.ino
 * @brief Interactive CLI demonstration for the ToyOS Key-Value Database.
 *
 * Provides a serial-based interface to interact with the persistent storage.
 * Supported commands:
 * - ADD k,v   : Write/Update a key with a value.
 * - DEL k     : Delete a key.
 * - SHOW      : List all database entries.
 * - LIST p    : List entries starting with prefix p.
 */

#include <Arduino.h>
#include <kv_db.h>
#include <toyos.h>

static uint8_t mem_pool[4096] __attribute__((aligned(8)));
Mutex serial_mutex;

void log_msg(const char *msg) {
  os_mutex_lock(&serial_mutex);
  Serial.println(msg);
  Serial.flush();
  os_mutex_unlock(&serial_mutex);
}

void task_blink(void) {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    os_delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    os_delay(500);
  }
}

void task_cli(void) {
  log_msg("ToyOS KV Database CLI Ready. Commands: ADD k,v | DEL k | SHOW | "
          "LIST prefix");
  static char cmd[64];
  static uint8_t pos = 0;

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (pos > 0) {
          cmd[pos] = '\0';
          if (strncmp(cmd, "ADD ", 4) == 0) {
            char *k = cmd + 4;
            char *v = strchr(k, ',');
            if (v) {
              *v = '\0';
              v++;
              if (kv_write(k, v, strlen(v)) == KV_SUCCESS)
                log_msg("OK: Written");
              else
                log_msg("Error: Write failed");
            }
          } else if (strncmp(cmd, "DEL ", 4) == 0) {
            char *k = cmd + 4;
            if (kv_delete(k) == KV_SUCCESS)
              log_msg("OK: Deleted");
            else
              log_msg("Error: Delete failed");
          } else if (strcmp(cmd, "SHOW") == 0) {
            os_mutex_lock(&serial_mutex);
            Serial.println("--- DB Content ---");
            kv_iterate(
                NULL,
                [](const char *k, const void *v, uint16_t l, void *ctx) {
                  Serial.print(k);
                  Serial.print(": ");
                  Serial.write((const uint8_t *)v, l);
                  Serial.println();
                },
                NULL);
            Serial.println("------------------");
            os_mutex_unlock(&serial_mutex);
          } else if (strncmp(cmd, "LIST ", 5) == 0) {
            char *prefix = cmd + 5;
            os_mutex_lock(&serial_mutex);
            Serial.print("--- Prefix '");
            Serial.print(prefix);
            Serial.println("' ---");
            kv_iterate(
                prefix,
                [](const char *k, const void *v, uint16_t l, void *ctx) {
                  Serial.print(k);
                  Serial.print(": ");
                  Serial.write((const uint8_t *)v, l);
                  Serial.println();
                },
                NULL);
            Serial.println("------------------");
            os_mutex_unlock(&serial_mutex);
          }
          pos = 0;
        }
      } else if (pos < 63) {
        if (c >= 32 && c <= 126)
          cmd[pos++] = c;
      }
    }
    os_delay(50);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  storage_bind_platform_driver();
  storage_init();
  kv_init();

  os_mutex_init(&serial_mutex);
  os_init(mem_pool, sizeof(mem_pool));

  os_create_task(1, task_blink, 1, 512);
  os_create_task(2, task_cli, 1, 1536);
  os_start();
}

void loop() {}
