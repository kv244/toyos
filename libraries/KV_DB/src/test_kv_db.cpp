#include "kv_db.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <toyos.h>

extern Mutex serial_mutex;

static void test_log(const char *msg, bool success) {
  os_mutex_lock(&serial_mutex);
  Serial.print(success ? F("[PASS] ") : F("[FAIL] "));
  Serial.println(msg);
  os_mutex_unlock(&serial_mutex);
}

/**
 * Basic CRUD operations test.
 */
void test_basic_crud(void) {
  const char *key = "test_key";
  const char *val = "test_value";
  char buffer[32];
  uint16_t len;

  kv_clear();

  // Write
  if (kv_write(key, val, strlen(val)) != KV_SUCCESS) {
    test_log("kv_write basic", false);
    return;
  }

  // Read
  if (kv_read(key, buffer, sizeof(buffer), &len) != KV_SUCCESS ||
      strncmp(buffer, val, len) != 0) {
    test_log("kv_read basic", false);
    return;
  }

  // Update
  const char *val2 = "updated_value";
  if (kv_write(key, val2, strlen(val2)) != KV_SUCCESS) {
    test_log("kv_write update", false);
    return;
  }
  if (kv_read(key, buffer, sizeof(buffer), &len) != KV_SUCCESS ||
      strncmp(buffer, val2, len) != 0) {
    test_log("kv_read update", false);
    return;
  }

  // Delete
  if (kv_delete(key) != KV_SUCCESS) {
    test_log("kv_delete", false);
    return;
  }
  if (kv_read(key, buffer, sizeof(buffer), &len) != KV_ERR_NOT_FOUND) {
    test_log("kv_read after delete", false);
    return;
  }

  test_log("Basic CRUD", true);
}

/**
 * Test persistence across "reboots" (re-initialization).
 */
void test_persistence(void) {
  kv_clear();
  kv_write("p_key", "p_val", 5);

  // Re-initialize (simulates reboot)
  if (kv_init() != KV_SUCCESS) {
    test_log("kv_init persistence", false);
    return;
  }

  char buffer[16];
  uint16_t len;
  if (kv_read("p_key", buffer, sizeof(buffer), &len) != KV_SUCCESS ||
      strncmp(buffer, "p_val", 5) != 0) {
    test_log("kv_read persistence", false);
    return;
  }

  test_log("Persistence", true);
}

/**
 * Test edge cases like long keys and values.
 */
void test_edge_cases(void) {
  // Key too long (> 24)
  const char *long_key = "this_key_is_waaaaay_too_long_for_the_db";
  if (kv_write(long_key, "val", 3) != KV_ERR_KEY_TOO_LONG) {
    test_log("Key too long limit", false);
  } else {
    test_log("Key too long limit", true);
  }

  // Value too long (> 1024)
  // Use a small actual buffer but tell kv_write it's 1025
  // (We don't actually call kv_write with the invalid pointer if we check
  // length first)
  if (kv_write("long_val_key", "dummy", 1025) != KV_ERR_VAL_TOO_LONG) {
    test_log("Value too long limit", false);
  } else {
    test_log("Value too long limit", true);
  }
}

/**
 * Test EEPROM full scenario.
 */
void test_eeprom_full(void) {
  kv_clear();
  char key[8];
  char val[] = "dummy_val"; // record size = header(6) + key(x) + val(9)

  // Fill until full
  while (true) {
    static uint8_t count = 0;
    key[0] = 'k';
    key[1] = '0' + (count / 10);
    key[2] = '0' + (count % 10);
    key[3] = '\0';
    count++;

    os_wdt_feed(); // Feed watchdog during long operation

    kv_result_t res = kv_write(key, val, strlen(val));
    if (res == KV_ERR_FULL) {
      test_log("EEPROM Full handling", true);
      break;
    } else if (res != KV_SUCCESS) {
      test_log("EEPROM Full handling (error)", false);
      break;
    }
  }
}

/**
 * Concurrency Test Tasks
 */
static volatile bool task1_done = false;
static volatile bool task2_done = false;

void task_writer1(void) {
  for (int i = 0; i < 5; i++) {
    kv_write("shared_k1", "val1", 4);
    os_delay(10);
  }
  task1_done = true;
  while (1)
    os_delay(1000);
}

void task_writer2(void) {
  for (int i = 0; i < 5; i++) {
    kv_write("shared_k2", "val2", 4);
    os_delay(10);
  }
  task2_done = true;
  while (1)
    os_delay(1000);
}

void test_concurrency(void) {
  kv_clear();
  task1_done = false;
  task2_done = false;

  // We expect these tasks to run concurrently without corruption due to Mutex
  os_create_task(3, task_writer1, 3, 140);
  os_create_task(4, task_writer2, 3, 140);

  // Wait for tasks to finish
  uint16_t timeout = 0;
  while (!task1_done || !task2_done) {
    os_delay(100);
    timeout++;
    if (timeout > 100) { // 10s timeout
      test_log("Concurrency (Timeout)", false);
      return;
    }
  }

  char b1[8], b2[8];
  uint16_t l1, l2;
  bool s1 = (kv_read("shared_k1", b1, 8, &l1) == KV_SUCCESS);
  bool s2 = (kv_read("shared_k2", b2, 8, &l2) == KV_SUCCESS);

  if (s1 && s2) {
    test_log("Concurrency", true);
  } else {
    test_log("Concurrency", false);
  }
}

/**
 * Test compaction functionality.
 */
void test_compaction(void) {
  kv_clear();

  // Create gaps by updating and deleting
  kv_write("k1", "v1", 2);
  kv_write("k2", "v2", 2);
  kv_write("k1", "v1_updated", 10); // Orphaning old k1
  kv_delete("k2");                  // Orphaning k2

  int16_t reclaimed = kv_compact();

  char buffer[16];
  uint16_t len;
  if (reclaimed > 0 &&
      kv_read("k1", buffer, sizeof(buffer), &len) == KV_SUCCESS &&
      strncmp(buffer, "v1_updated", 10) == 0 &&
      kv_read("k2", buffer, sizeof(buffer), &len) == KV_ERR_NOT_FOUND) {
    test_log("Compaction", true);
  } else {
    test_log("Compaction", false);
  }
}

extern "C" void run_all_tests(void) {
  os_mutex_lock(&serial_mutex);
  Serial.println(F("--- Starting DB Test Suite ---"));
  os_mutex_unlock(&serial_mutex);

  test_basic_crud();
  test_persistence();
  test_edge_cases();
  test_eeprom_full();
  test_compaction();
  test_concurrency();

  os_mutex_lock(&serial_mutex);
  Serial.println(F("--- DB Test Suite Finished ---"));
  os_mutex_unlock(&serial_mutex);
}
