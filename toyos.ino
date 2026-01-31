#include "toyos.h"
#include <Arduino.h>
#include <avr/wdt.h>

static uint8_t mem_pool[832] __attribute__((aligned(2)));

Mutex serial_mutex;
Mutex pi_mutex;

/* Priority Inheritance Test Tasks */
void task_low(void) {
  os_mutex_lock(&pi_mutex);

  os_mutex_lock(&serial_mutex);
  Serial.print(F("[PI] Low locked. Base Prio: 1, Current Prio: "));
  Serial.println(os_get_priority(20));
  os_mutex_unlock(&serial_mutex);

  /* Long busy wait. During this:
   * 1. Med (2) will preempt and run.
   * 2. High (3) will preempt Med and try to lock.
   * 3. High will boost Low to 3.
   * 4. Low (3) will preempt Med (2) and finish its work.
   */
  for (volatile uint32_t i = 0; i < 1500000; i++)
    ;

  os_mutex_lock(&serial_mutex);
  Serial.print(F("[PI] Low finished loop. Current Prio: "));
  Serial.println(os_get_priority(20));
  Serial.print(F("[PI] Stack Usage (Low): "));
  Serial.println(os_get_stack_usage(20));
  os_mutex_unlock(&serial_mutex);

  os_mutex_unlock(&pi_mutex);

  os_mutex_lock(&serial_mutex);
  Serial.print(F("[PI] Low released mutex. Restored Prio: "));
  Serial.println(os_get_priority(20));
  os_mutex_unlock(&serial_mutex);

  while (1)
    os_delay(1000);
}

void task_med(void) {
  os_delay(100); // Start after Low locks
  os_mutex_lock(&serial_mutex);
  Serial.println(F("[PI] Med woke up and preempted Low."));
  os_mutex_unlock(&serial_mutex);

  while (1) {
    /* Busy loop to show priority levels.
     * If PI works, Low will preempt this even though Low's base is 1 < 2.
     * If PI FAILS, this will run forever and High will never get the lock.
     */
    os_wdt_feed(); // Specifically feed here so it doesn't reset during Med's
                   // turn
  }
}

void task_high(void) {
  os_delay(200); // Start after Med
  os_mutex_lock(&serial_mutex);
  Serial.println(F("[PI] High woke up and trying to lock mutex..."));
  os_mutex_unlock(&serial_mutex);

  os_mutex_lock(&pi_mutex);

  os_mutex_lock(&serial_mutex);
  Serial.println(F("[PI] High GOT lock! PI Success."));
  Serial.print(F("[PI] Stack Usage (High): "));
  Serial.println(os_get_stack_usage(22));
  os_mutex_unlock(&serial_mutex);

  os_mutex_unlock(&pi_mutex);
  while (1)
    os_delay(1000);
}

void task_idle(void) {
  static uint16_t idle_count = 0;
  while (1) {
    os_wdt_feed();
    os_check_stack_overflow();

    if (++idle_count >= 1000) {
      os_mutex_lock(&serial_mutex);
      Serial.println(F("[IDLE] System free."));
      os_mutex_unlock(&serial_mutex);
      idle_count = 0;
    }

    os_enter_idle();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println(F("ToyOS V2.4 - Priority Inheritance Test"));
  Serial.println(F("======================================="));
  Serial.flush();

  os_init(mem_pool, sizeof(mem_pool));
  os_mutex_init(&serial_mutex);
  os_mutex_init(&pi_mutex);

  /* Init Watchdog - 8s for safe PI test */
  os_wdt_init(WDTO_8S);

  /* Create tasks: L(1), M(2), H(3) */
  os_create_task(20, task_low, 1, 144);
  os_create_task(21, task_med, 2, 144);
  os_create_task(22, task_high, 3, 144);
  os_create_task(0, task_idle, 0, 80);

  os_start();
}

void loop() {}
