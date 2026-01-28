#include "toyos.h"
#include <Arduino.h>

/* Memory pool for OS (1KB) */
static uint8_t mem_pool[1024];

/* Shared resources */
Mutex led_mutex;
MessageQueue *sensor_mq;

/* Global state for display */
volatile int shared_data = 0;

/* LED Pin */
#define LED_PIN 13

/* Task 1: Sensor Producer (High Priority)
 * Periodically reads a sensor and sends data to a queue.
 */
void task_producer(void) {
  static int count = 0;
  while (1) {
    int val = analogRead(A0);

    /* Simulate some processing */
    count++;

    /* Send pointer to value/data (for demo, we'll just cast the value) */
    os_mq_send(sensor_mq, (void *)(uintptr_t)val);

    Serial.print(F("[Producer] Sent: "));
    Serial.println(val);

    os_delay(1000); /* Read every 1 second */
  }
}

/* Task 2: Data Consumer (Medium Priority)
 * Waits for data and prints it.
 */
void task_consumer(void) {
  while (1) {
    void *msg = os_mq_receive(sensor_mq);
    int val = (int)(uintptr_t)msg;

    Serial.print(F("[Consumer] Processed: "));
    Serial.println(val);

    /* Access shared resource using Mutex */
    os_mutex_lock(&led_mutex);
    shared_data = val;
    os_mutex_unlock(&led_mutex);
  }
}

/* Task 3 & 4: Mutex Competition tasks (Low Priority)
 * Both tasks try to toggle the same LED.
 */
void task_led_a(void) {
  while (1) {
    os_mutex_lock(&led_mutex);
    Serial.println(F("Task A has LED Mutex"));
    digitalWrite(LED_PIN, HIGH);
    os_delay(200);
    digitalWrite(LED_PIN, LOW);
    os_mutex_unlock(&led_mutex);

    os_delay(500);
  }
}

void task_led_b(void) {
  while (1) {
    os_mutex_lock(&led_mutex);
    Serial.println(F("Task B has LED Mutex"));
    digitalWrite(LED_PIN, HIGH);
    os_delay(100);
    digitalWrite(LED_PIN, LOW);
    os_mutex_unlock(&led_mutex);

    os_delay(300);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println(F("ToyOS V2.0 - True Context Switching Demo"));
  Serial.println(F("========================================"));

  pinMode(LED_PIN, OUTPUT);

  /* Initialize OS and primitives */
  os_init(mem_pool, sizeof(mem_pool));
  os_mutex_init(&led_mutex);
  sensor_mq = os_mq_create(5); /* Capacity of 5 messages */

  /* Create tasks:
   * (ID, Func, Priority, StackSize)
   * Higher priority = higher number? (Based on heap logic: bigger first)
   */
  os_create_task(10, task_producer, 10, 128); /* Highest priority */
  os_create_task(5, task_consumer, 5, 128);   /* Medium priority */
  os_create_task(1, task_led_a, 1, 128);      /* Low priority */
  os_create_task(2, task_led_b, 1, 128);      /* Low priority */

  Serial.println(F("Starting Pre-emptive Scheduler..."));
  os_start();
}

void loop() {}
