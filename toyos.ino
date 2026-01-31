#include "toyos.h"
#include <Arduino.h>

// Use a larger memory pool for a more thorough test.
// Reduced memory pool for better global SRAM availability.
static uint8_t mem_pool[896] __attribute__((aligned(2)));

Mutex serial_mutex;

#define NUM_POINTERS 12

/*
 * This task stress-tests the buddy memory allocator.
 * It repeatedly allocates and frees blocks of random sizes to check for
 * correctness, fragmentation resistance, and memory corruption.
 */
void task_mem_test(void) {
  void *pointers[NUM_POINTERS];
  uint16_t sizes[NUM_POINTERS];

  os_mutex_lock(&serial_mutex);
  Serial.println(F("[MEMTEST] Memory test task starting."));
  os_mutex_unlock(&serial_mutex);

  // Initialize pointer array
  for (int i = 0; i < NUM_POINTERS; i++) {
    pointers[i] = NULL;
    sizes[i] = 0;
  }

  // Use an unconnected analog pin for a random seed
  randomSeed(analogRead(A0));

  // Run a number of allocation/deallocation cycles
  for (int cycle = 0; cycle < 100; cycle++) {
    int i = random(NUM_POINTERS); // Pick a random slot

    if (pointers[i] != NULL) {
      // Slot is in use, so free it.
      // First, verify its contents.
      bool ok = true;
      for (uint16_t j = 0; j < sizes[i]; j++) {
        if (((uint8_t *)pointers[i])[j] != (uint8_t)i) {
          ok = false;
          break;
        }
      }

      if (!ok) {
        os_mutex_lock(&serial_mutex);
        Serial.println(F("!!! CORRUPTION DETECTED !!!"));
        os_mutex_unlock(&serial_mutex);
        // Stop test on corruption
        while (1)
          os_delay(5000);
      }

      os_free(pointers[i]);

      os_mutex_lock(&serial_mutex);
      Serial.print(cycle);
      Serial.print(F(": Freed block in slot "));
      Serial.println(i);
      os_mutex_unlock(&serial_mutex);

      pointers[i] = NULL;
      sizes[i] = 0;

    } else {
      // Slot is free, so allocate a new block.
      uint16_t size = random(8, 128);
      pointers[i] = os_malloc(size);

      os_mutex_lock(&serial_mutex);
      Serial.print(cycle);
      if (pointers[i] != NULL) {
        sizes[i] = size;
        // Fill the new block with a known pattern (based on its index)
        memset(pointers[i], (uint8_t)i, size);
        Serial.print(F(": Allocated "));
        Serial.print(size);
        Serial.print(F(" bytes for slot "));
        Serial.println(i);
      } else {
        Serial.print(F(": FAILED to allocate "));
        Serial.print(size);
        Serial.println(F(" bytes."));
      }
      os_mutex_unlock(&serial_mutex);
    }
    os_delay(50); // Small delay between operations
  }

  os_mutex_lock(&serial_mutex);
  Serial.println(F("[MEMTEST] Test finished successfully."));
  os_mutex_unlock(&serial_mutex);

  while (1) {
    os_delay(1000);
  }
}

void task_idle(void) {
  while (1) {
    os_check_stack_overflow();
    os_enter_idle();
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  delay(1000); // Wait for serial to connect

  Serial.println(F("ToyOS V2.5 - Buddy Allocator Stress Test"));
  Serial.println(F("========================================="));
  Serial.flush();

  os_init(mem_pool, sizeof(mem_pool));
  os_mutex_init(&serial_mutex);

  os_create_task(1, task_mem_test, 1, 256);
  os_create_task(0, task_idle, 0, 80);

  os_start();
}

void loop() {
  // Should not be reached.
}