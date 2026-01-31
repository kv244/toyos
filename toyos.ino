#include "toyos.h"
#include <Arduino.h>

static uint8_t mem_pool[1024] __attribute__((aligned(2)));

Mutex serial_mutex;
MessageQueue *sensor_mq;

#define LED_PIN 13
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_BIT PB5
#define LED_ON() (LED_PORT |= (1 << LED_BIT))
#define LED_OFF() (LED_PORT &= ~(1 << LED_BIT))

/* TEST: Global Semaphore for LED synchronization */
Semaphore led_sem;

void task_producer(void) {
  int counter = 0;

  while (1) {
    /* TEST: Malloc/Free Stress Test - Allocate 32 bytes */
    char *dynamic_buf = (char *)os_malloc(32);
    if (dynamic_buf) {
      strcpy(dynamic_buf, "Malloc: OK");
      // Prevent compiler optimization by reading it back
      if (dynamic_buf[0] != 'M')
        Serial.println(F("Error"));

      /* Valid allocation - now free it */
      os_free(dynamic_buf);
    } else {
      /* Leak detection! */
      Serial.println(F("Malloc: FAILED (Leak?)"));
    }

    /* Send counter value as a pointer (cast) - USING FAST PATH */
    os_mq_send_fast(sensor_mq, (void *)(uintptr_t)counter);

    /* TEST: Mutex for Serial & GetTick */
    os_mutex_lock(&serial_mutex);
    Serial.print(F("Prod Sent: "));
    Serial.print(counter);
    Serial.print(F(" @ Tick: "));
    Serial.println(os_get_tick());

    if (!dynamic_buf)
      Serial.println(F("STATUS: LEAKING MEMORY!"));

    os_mutex_unlock(&serial_mutex);

    counter++;
    os_delay(1000);
  }
}

void task_consumer_1(void) {
  while (1) {
    void *msg = os_mq_receive_fast(sensor_mq);
    int val = (int)(uintptr_t)msg;

    os_mutex_lock(&serial_mutex);
    Serial.print(F("Cons1 Got (Fast): "));
    Serial.println(val);
    os_mutex_unlock(&serial_mutex);

    /* TEST: Cooperative Yield */
    os_task_yield();

    os_delay(500);
  }
}

void task_consumer_2(void) {
  while (1) {
    void *msg = os_mq_receive_fast(sensor_mq);
    int val = (int)(uintptr_t)msg;

    os_mutex_lock(&serial_mutex);
    Serial.print(F("Cons2 Got (Fast): "));
    Serial.println(val);
    os_mutex_unlock(&serial_mutex);

    os_delay(500);
  }
}

void task_led_a(void) {
  while (1) {
    /* Blink Pattern A */
    LED_ON();
    os_delay(200);
    LED_OFF();

    /* TEST: Signal Task B to run now */
    os_sem_post(&led_sem);

    os_delay(800);
  }
}

void task_led_b(void) {
  while (1) {
    /* TEST: Wait for Signal from Task A */
    os_sem_wait(&led_sem);

    /* Fast double-blink to distinguish */
    LED_ON();
    os_delay(50);
    LED_OFF();
    os_delay(50);
    LED_ON();
    os_delay(50);
    LED_OFF();
  }
}

void task_idle(void) {
  while (1) {
    os_check_stack_overflow();
    os_enter_idle();
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;

  Serial.println(F("ToyOS V2.2 - Comprehensive Test Suite"));
  Serial.println(F("====================================="));
  Serial.flush();

  /* Configure LED pin */
  LED_DDR |= (1 << LED_BIT);
  LED_OFF();

  /* Initialize OS and primitives */
  os_init(mem_pool, sizeof(mem_pool));
  Serial.println(F("OS Init: OK"));

  os_mutex_init(&serial_mutex);
  os_sem_init(&led_sem, 0); /* Init semaphore with 0 count (locked) */

  sensor_mq = os_mq_create(5);
  if (sensor_mq == NULL) {
    Serial.println(F("MQ Create: FAILED"));
    while (1)
      ;
  }
  Serial.println(F("MQ Create: OK"));

  /* Create tasks */
  os_create_task(10, task_producer, 10, 256);
  Serial.println(F("Task Prod: OK"));

  os_create_task(2, task_consumer_1, 5, 128);
  Serial.println(F("Task Cons1: OK"));

  os_create_task(3, task_consumer_2, 5, 128);
  Serial.println(F("Task Cons2: OK"));

  os_create_task(1, task_led_a, 1, 80);
  os_create_task(1, task_led_b, 1, 80);
  os_create_task(0, task_idle, 0, 64);
  Serial.println(F("All Tasks: OK"));

  Serial.println(F("Starting Pre-emptive Scheduler..."));
  Serial.flush();
  os_start();

  while (1) {
    // Should never reach here
  }
}

void loop() {}
