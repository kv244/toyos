/**
 * @file KVTestRunner.ino
 * @brief Automated test suite for the ToyOS Key-Value Database.
 *
 * Runs a series of regression tests covering CRUD operations, prefix iteration,
 * and database clearing. Results are reported via Serial.
 */

#include <Arduino.h>
#include <kv_db.h>
#include <toyos.h>

static uint8_t mem_pool[8192] __attribute__((aligned(8)));

// Test Helpers
#define ASSERT_KV(res, expected)                                               \
  if (res != expected) {                                                       \
    Serial.print("  [FAIL] Expected result ");                                 \
    Serial.print(expected);                                                    \
    Serial.print(", got ");                                                    \
    Serial.println(res);                                                       \
    return false;                                                              \
  }

void run_test(const char *name, bool (*test_func)()) {
  Serial.print("[TEST] ");
  Serial.print(name);
  Serial.print("... ");
  if (test_func()) {
    Serial.println("PASS");
  } else {
    Serial.println("FAILED");
  }
}

// Test Cases

bool test_write_read() {
  const char *key = "test_key";
  const char *val = "Hello ToyOS";
  char buffer[32];
  uint16_t len;

  ASSERT_KV(kv_write(key, val, strlen(val)), KV_SUCCESS);
  ASSERT_KV(kv_read(key, buffer, sizeof(buffer), &len), KV_SUCCESS);

  buffer[len] = '\0';
  if (strcmp(buffer, val) != 0) {
    Serial.println("  [FAIL] Data mismatch");
    return false;
  }
  return true;
}

bool test_delete() {
  const char *key = "del_key";
  char buffer[16];

  kv_write(key, "data", 4);
  ASSERT_KV(kv_delete(key), KV_SUCCESS);
  ASSERT_KV(kv_read(key, buffer, sizeof(buffer), NULL), KV_ERR_NOT_FOUND);
  return true;
}

static int iter_count = 0;
void iter_cb(const char *key, const void *val, uint16_t val_len, void *ctx) {
  iter_count++;
}

bool test_iterate() {
  Serial.println("  Clearing DB...");
  kv_clear();
  Serial.println("  Writing test records...");
  kv_write("users/alice", "1", 1);
  kv_write("users/bob", "2", 1);
  kv_write("config/wifi", "on", 2);

  iter_count = 0;
  Serial.println("  Iterating all...");
  ASSERT_KV(kv_iterate("", iter_cb, NULL), KV_SUCCESS);
  if (iter_count != 3) {
    Serial.print("  [FAIL] Expected 3 items, got ");
    Serial.println(iter_count);
    return false;
  }

  iter_count = 0;
  Serial.println("  Iterating prefix 'users/'...");
  ASSERT_KV(kv_iterate("users/", iter_cb, NULL), KV_SUCCESS);
  if (iter_count != 2) {
    Serial.print("  [FAIL] Expected 2 items for prefix 'users/', got ");
    Serial.println(iter_count);
    return false;
  }

  return true;
}

void task_test_runner(void) {
  os_delay(2000); // Give user time to see the start
  Serial.println("\n--- Starting KV Database Automated Tests ---");

  run_test("Write/Read Operation", test_write_read);
  run_test("Delete Operation", test_delete);
  run_test("Iterate Operation", test_iterate);

  Serial.println("--- All Tests Completed ---");

  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    os_delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    os_delay(900);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Light up LED while waiting

  Serial.begin(115200);
  // Be very patient for USB Serial on R4
  for (int i = 0; i < 50 && !Serial; i++) {
    delay(100);
  }

  Serial.println("ToyOS KV Test Runner Booting...");

  // Init HAL & KV
  Serial.print("Initializing Storage... ");
  storage_bind_platform_driver();
  if (storage_init() == STORAGE_OK) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  Serial.print("Initializing KV DB... ");
  if (kv_init() == KV_SUCCESS) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // Init OS
  Serial.print("Initializing ToyOS... ");
  os_init(mem_pool, sizeof(mem_pool));
  Serial.println("OK");

  // Create test runner task
  os_create_task(1, task_test_runner, 1, 2048);

  Serial.println("Starting Scheduler. Tests will begin in 2 seconds...");
  os_start();
}

void loop() {}
