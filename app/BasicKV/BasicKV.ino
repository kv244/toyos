#include <Arduino.h>
#include <kv_db.h>
#include <toyos.h>


static uint8_t mem_pool[8192] __attribute__((aligned(8)));

void task_counter(void) {
  uint32_t counter = 0;
  uint16_t len;

  // Try to load existing counter
  if (kv_read("boot_count", (char *)&counter, sizeof(counter), &len) ==
      KV_SUCCESS) {
    Serial.print("[KV] Last boot count found: ");
    Serial.println(counter);
  } else {
    Serial.println("[KV] No previous boot count found. Starting at 0.");
    counter = 0;
  }

  // Increment and save
  counter++;
  if (kv_write("boot_count", (const char *)&counter, sizeof(counter)) ==
      KV_SUCCESS) {
    Serial.println("[KV] New boot count saved.");
  } else {
    Serial.println("[KV] Error: Failed to save boot count.");
  }

  while (1) {
    Serial.print("Uptime counter: ");
    Serial.println(counter++);

    // Periodically save to KV every 10 counts
    if (counter % 10 == 0) {
      if (kv_write("uptime", (const char *)&counter, sizeof(counter)) ==
          KV_SUCCESS) {
        Serial.println("[KV] Uptime checkpoint saved.");
      }
    }

    os_delay(1000);
  }
}

void task_blink(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    os_delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    os_delay(200);
  }
}

void setup() {
  Serial.begin(115200);
  // Wait for Serial to be ready (critical for R4 WiFi)
  unsigned long start = millis();
  while (!Serial && (millis() - start < 3000))
    ;

  Serial.println("\n--- ToyOS KV Simple Demo ---");

  // Initialize KV storage drivers
  storage_bind_platform_driver();
  if (storage_init() != STORAGE_OK) {
    Serial.println("Storage HAL Init Failed!");
  }

  if (kv_init() != KV_SUCCESS) {
    Serial.println("KV Database Init Failed! (Formatting if first run)");
  }

  // Initialize ToyOS
  os_init(mem_pool, sizeof(mem_pool));

  // Create tasks
  os_create_task(1, task_blink, 1, 512);
  os_create_task(2, task_counter, 1, 2048);

  Serial.println("Starting Scheduler...");
  os_start();
}

void loop() {
  // ToyOS takes over
}
