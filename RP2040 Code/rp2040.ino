#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>

//Encoder
const int CLOCK_PIN1 = 8;
const int DATA_PIN1 = 7;
const int BIT_COUNT = 12;

//Stepper motor
const int ENABLE_PIN = 27;
const int STEP_PIN = 28;
const int DIR_PIN = 29;

int previous_angle = 0;
long turn_count = 0;
int zero_offset = 0;

void readEncoderTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10 ms = 100 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    int read = shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT);
    int raw_angle = ((read & 0x0FFF) * 360.0) / 4096.0;

    // Adjust to zero offset captured in setup()
    int current_angle = raw_angle - zero_offset;
    if (current_angle < 0) current_angle += 360;

    // Multi-turn tracking
    int delta = current_angle - previous_angle;
    if (delta > 180) {
      turn_count--;
    } else if (delta < -180) {
      turn_count++;
    }

    previous_angle = current_angle;

    // Total angle and gearbox output
    float input_total_angle = turn_count * 360.0 + current_angle;
    float output_angle = fmod((input_total_angle / 5.0), 360.0);
    if (output_angle < 0) output_angle += 360.0;

    Serial.println(output_angle);
  }
}

void setup() {
  pinMode(DATA_PIN1, INPUT);
  pinMode(CLOCK_PIN1, OUTPUT);
  digitalWrite(CLOCK_PIN1, HIGH);

  Serial.begin(115200);
  delay(50);

  // Read encoder once to define zero offset
  int read = shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT);
  zero_offset = ((read & 0x0FFF) * 360.0) / 4096.0;
  previous_angle = 0;

  xTaskCreate(
    readEncoderTask,
    "EncoderReader",
    2048,
    NULL,
    1,
    NULL
  );
}

void loop() {
  // Nothing here; task handles it
}

unsigned long shiftIn(const int data_pin, const int clock_pin, const int bit_count) {
  unsigned long data = 0;
  for (int i = 0; i < bit_count; i++) {
    data <<= 1;
    digitalWrite(clock_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(clock_pin, HIGH);
    delayMicroseconds(1);
    data |= digitalRead(data_pin);
  }
  return data;
}
