#include "toyos.h"
#include <Arduino.h>

/* Memory pool for OS (1KB) */
static uint8_t mem_pool[1024];

/* Shared resources */
Mutex led_mutex;
MessageQueue *sensor_mq;

/* Global state for display */
volatile int shared_data = 0;

/* LED Pin - Direct port manipulation for performance (OPTIMIZATION #11) */
#define LED_PIN 13
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_BIT PB5  /* Pin 13 on Arduino UNO */

/* Fast LED macros - saves ~50 CPU cycles per call */
#define LED_ON()  (LED_PORT |= (1 << LED_BIT))
#define LED_OFF() (LED_PORT &= ~(1 << LED_BIT))
#define LED_TOGGLE() (LED_PORT ^= (1 << LED_BIT))

/* Task 1: Sensor Producer (High Priority)
 * Periodically reads a sensor and sends data to a queue.
 */
void task_producer(void) {
  static int count = 0;
  while (1) {
    int val = analogRead(A0);

    /* Simulate some processing */
    count++;

    /* Use fast message queue - fewer context switches (OPTIMIZATION #14) */
    os_mq_send_fast(sensor_mq, (void *)(uintptr_t)val);

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
    /* Use fast message queue - fewer context switches (OPTIMIZATION #14) */
    void *msg = os_mq_receive_fast(sensor_mq);
    int val = (int)(uintptr_t)msg;

    Serial.print(F("[Consumer] Processed: "));
    Serial.println(val);

    /* Access shared resource using Mutex */
    os_mutex_lock(&led_mutex);
    shared_data = val;
    os_mutex_unlock(&led_mutex);
  }
}

/* Task 3: Mutex Competition task A (Low Priority)
 * Tries to toggle LED - uses direct port manipulation
 */
void task_led_a(void) {
  while (1) {
    os_mutex_lock(&led_mutex);
    Serial.println(F("Task A has LED Mutex"));
    
    /* Direct port manipulation - much faster than digitalWrite() */
    LED_ON();
    os_delay(200);
    LED_OFF();
    
    os_mutex_unlock(&led_mutex);

    os_delay(500);
  }
}

/* Task 4: Mutex Competition task B (Low Priority)
 * Tries to toggle LED - uses direct port manipulation
 */
void task_led_b(void) {
  while (1) {
    os_mutex_lock(&led_mutex);
    Serial.println(F("Task B has LED Mutex"));
    
    /* Direct port manipulation - much faster than digitalWrite() */
    LED_ON();
    os_delay(100);
    LED_OFF();
    
    os_mutex_unlock(&led_mutex);

    os_delay(300);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println(F("ToyOS V2.1 - FIXED & OPTIMIZED"));
  Serial.println(F("================================"));
  Serial.println(F("Fixes:"));
  Serial.println(F("- Stack corruption fix"));
  Serial.println(F("- Semaphore/mutex context switch fix"));
  Serial.println(F("- Scheduler blocked task handling"));
  Serial.println(F("- Memory leak prevention"));
  Serial.println(F("- Stack overflow detection"));
  Serial.println(F("- uint32_t system tick"));
  Serial.println(F("\nOptimizations:"));
  Serial.println(F("- Fast message queue operations"));
  Serial.println(F("- Direct port manipulation"));
  Serial.println(F("- Assembly stack zeroing"));
  Serial.println(F("- Optimized atomic operations"));
  Serial.println();

  /* Configure LED pin using direct register access */
  LED_DDR |= (1 << LED_BIT);  /* Set as output */
  LED_OFF();                   /* Start with LED off */

  /* Initialize OS and primitives */
  os_init(mem_pool, sizeof(mem_pool));
  os_mutex_init(&led_mutex);
  sensor_mq = os_mq_create(5); /* Capacity of 5 messages */

  /* Create tasks:
   * (ID, Func, Priority, StackSize)
   * Higher priority number = higher priority
   */
  os_create_task(10, task_producer, 10, 128); /* Highest priority */
  os_create_task(5, task_consumer, 5, 128);   /* Medium priority */
  os_create_task(1, task_led_a, 1, 96);       /* Low priority - reduced stack */
  os_create_task(2, task_led_b, 1, 96);       /* Low priority - reduced stack */

  Serial.println(F("Starting Pre-emptive Scheduler..."));
  Serial.print(F("Free memory: "));
  Serial.print(1024 - ((uint16_t)&mem_pool[1024] - (uint16_t)mem_pool));
  Serial.println(F(" bytes"));
  
  os_start();
}

void loop() {
  /* Never reached - OS takes over */
}
