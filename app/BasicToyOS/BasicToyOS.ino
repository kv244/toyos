#include <Arduino.h>
#include <toyos.h>

static uint8_t mem_pool[4096] __attribute__((aligned(8)));

void task_1(void) {
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    os_delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    os_delay(500);
  }
}

void task_2(void) {
  while (1) {
    Serial.println("System Alive...");
    os_delay(5000);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(1, task_1, 1, 1024);
  os_create_task(2, task_2, 1, 1024);
  os_start();
}

void loop() {}
