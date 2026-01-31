#include "toyos.h"
#include <Arduino.h>

static uint8_t mem_pool[600];

Mutex led_mutex;
Mutex serial_mutex;
MessageQueue *sensor_mq;
volatile int shared_data = 0;

#define LED_PIN 13
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_BIT PB5
#define LED_ON() (LED_PORT |= (1 << LED_BIT))
#define LED_OFF() (LED_PORT &= ~(1 << LED_BIT))

void task_producer(void) {
  int counter = 0;
  while (1) {
    /* Send counter value as a pointer (cast) - USING FAST PATH */
    os_mq_send_fast(sensor_mq, (void *)(uintptr_t)counter);
    Serial.print(F("Prod Sent: "));
    Serial.println(counter);
    counter++;
    os_delay(1000);
  }
}

void task_consumer_1(void) {
  while (1) {
    void *msg = os_mq_receive_fast(sensor_mq);
    int val = (int)(uintptr_t)msg;
    Serial.print(F("Cons1 Got (Fast): "));
    Serial.println(val);
    /* Simulate work */
    os_delay(500);
  }
}

void task_consumer_2(void) {
  while (1) {
    void *msg = os_mq_receive_fast(sensor_mq);
    int val = (int)(uintptr_t)msg;
    Serial.print(F("Cons2 Got (Fast): "));
    Serial.println(val);
    /* Simulate work */
    os_delay(500);
  }
}

void task_led_a(void) {
  while (1) {
    LED_ON();
    os_delay(200);
    LED_OFF();
    os_delay(500);
  }
}

void task_led_b(void) {
  while (1) {
    LED_ON();
    os_delay(100);
    LED_OFF();
    os_delay(300);
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

  Serial.println(F("ToyOS V2.2 - Multi-Consumer Demo (FAST MQ)"));
  Serial.println(F("================================"));
  Serial.flush();

  /* Configure LED pin using direct register access */
  LED_DDR |= (1 << LED_BIT);
  LED_OFF();

  /* Initialize OS and primitives */
  os_init(mem_pool, sizeof(mem_pool));
  os_mutex_init(&led_mutex);
  os_mutex_init(&serial_mutex);
  sensor_mq = os_mq_create(5);

  /* Create tasks */
  os_create_task(10, task_producer, 10, 100);
  os_create_task(2, task_consumer_1, 5, 100);
  os_create_task(3, task_consumer_2, 5, 100);
  os_create_task(1, task_led_a, 1, 80);
  os_create_task(1, task_led_b, 1, 80);
  os_create_task(0, task_idle, 0, 64);

  Serial.println(F("Starting Pre-emptive Scheduler..."));
  Serial.flush();
  os_start();

  while (1) {
    // Should never reach here
  }
}

void loop() {}
