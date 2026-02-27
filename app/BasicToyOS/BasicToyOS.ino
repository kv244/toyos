#include <Arduino.h>
#include <toyos.h>

static uint8_t mem_pool[4096] __attribute__((aligned(8)));

void task_1(void) {
  Serial.println("TASK 1: Running");
  Serial.flush();
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    os_delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    os_delay(500);
    Serial.println("TASK 1: Loop");
    Serial.flush();
  }
}

void task_2(void) {
  Serial.println("TASK 2: Running");
  Serial.flush();
  uint32_t count = 0;
  while (1) {
    os_delay(1000);
    count++;
    Serial.print("TASK 2: Count=");
    Serial.println(count);
    Serial.flush();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(1000);

  Serial.println("TOYOS STARTING...");
  Serial.flush();

  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(1, task_1, 1, 1024);
  os_create_task(2, task_2, 1, 1024);
  os_start();
}

void loop() {}
