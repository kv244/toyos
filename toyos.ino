/*
 * ToyOS - A Simple RTOS for Arduino UNO R3
 * Main application file (Arduino-compatible)
 */

#include "toyos.h"
#include <Arduino.h>


/* Memory pool for OS (768 bytes, leaving room for stack) */
static uint8_t mem_pool[768];

/* Task counters */
volatile uint16_t blink_counter = 0;
volatile uint16_t sensor_counter = 0;
volatile uint16_t compute_counter = 0;

/* LED Pin */
#define LED_PIN 13

/* Task 1: LED Blink Task */
void task_blink(void) {
  blink_counter++;

  /* Toggle LED */
  static uint8_t led_state = 0;
  led_state = !led_state;
  digitalWrite(LED_PIN, led_state);

  Serial.print(F("Task 1 (Blink): Counter = "));
  Serial.println(blink_counter);

  os_delay(500); /* Block for 500 ticks (500ms) */
}

/* Task 2: Simulated Sensor Task */
void task_sensor(void) {
  sensor_counter++;

  /* Read analog sensor (A0) */
  int sensor_value = analogRead(A0);

  Serial.print(F("Task 2 (Sensor): Counter = "));
  Serial.print(sensor_counter);
  Serial.print(F(", Value = "));
  Serial.println(sensor_value);

  os_delay(250); /* Block for 250 ticks (250ms) */
}

/* Task 3: Compute Task */
void task_compute(void) {
  compute_counter++;

  /* Simple computation example */
  static uint32_t sum = 0;
  sum += compute_counter;

  Serial.print(F("Task 3 (Compute): Iteration = "));
  Serial.print(compute_counter);
  Serial.print(F(", Sum = "));
  Serial.println(sum);

  os_task_yield(); /* Voluntary yield */
}

void setup() {
  /* Initialize Serial */
  Serial.begin(115200);
  while (!Serial) {
    ; /* Wait for serial port to connect (needed for native USB) */
  }

  Serial.println(F("ToyOS - Arduino UNO R3"));
  Serial.println(F("Initializing..."));

  /* Initialize LED pin */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  /* Initialize OS */
  os_init(mem_pool, sizeof(mem_pool));

  /* Create tasks with different priorities */
  os_create_task(1, task_blink, 1, 64);   /* LED blink task */
  os_create_task(2, task_sensor, 2, 64);  /* Sensor task */
  os_create_task(3, task_compute, 3, 64); /* Compute task */

  Serial.println(F("Starting scheduler..."));
  Serial.println(F("---"));

  /* Start the OS scheduler (never returns) */
  os_start();
}

void loop() { /* Never reached - os_start() runs forever */ }
