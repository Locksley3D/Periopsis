#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
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

//Encoder data
int previous_angle = 0;
long turn_count = 0;
int zero_offset = 0;

//Commands array and commands count
int commands[8];  
int command_count = 0;

volatile bool scanning = false;

//LED Configuration
#define LED_PIN     5      // GPIO0
#define NUM_LEDS    22    // Total number of WS2812B LEDs

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void readEncoderTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10 ms = 100 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int loop_counter = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); // Run every 10 ms

    int read = shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT); // Read bits
    int raw_angle = ((read & 0x0FFF) * 360.0) / 4096.0; // 12-bit to degrees

    // Adjust to zero offset
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

    //Increment loop counter and print every 50 ms
    if(scanning==true){
      loop_counter++;
      if (loop_counter >= 15) { // 10ms * 5 = 50ms
        Serial1.print("1,");
        Serial1.println(int(output_angle * 10)); // *10 to avoid decimals
        loop_counter = 0;
      }
    }
  }
}

void serialCommandTask(void *pvParameters) {
  String input = "";

  for (;;) {
    if (Serial1.available()) {
      char c = Serial1.read();//It may not seem like it but this loops until every character is in the input string ;)

      if (c == '\n') {
        // End of line: parse it
        command_count = 0;
        int value = 0;
        bool parsing = false;

        for (int i = 0; i < input.length() && command_count < 8; i++) {
          char ch = input[i];

          if (isdigit(ch)) {
            value = value * 10 + (ch - '0');
            parsing = true;
          } else if (ch == ',' && parsing) {
            commands[command_count++] = value;
            value = 0;
            parsing = false;
          }
        }

        // Print parsed commands
        Serial.print("Parsed Commands: ");
        for (int i = 0; i < command_count; i++) {
          Serial.print(commands[i]);
          Serial.print(" ");
        }
        Serial.println();

        input = ""; // Clear for next message
      } else {
        input += c; // Accumulate characters
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // Light delay to avoid busy-waiting
  }
}

void requestHandlerTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10 ms = 100 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  int steps = 0;
  int timeDelay = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if(commands[0]!=0){

      switch(commands[0]){
        case 1: //Base movement
          scanning = true;
          digitalWrite(ENABLE_PIN, LOW); //Turn the stepper motor on

      

          //Half turn of full turn
          if(commands[2]==1)steps = 4000;
          else if(commands[2]==2)steps = 8000;

          steps*=commands[1]; //scan cycle multiplier, multiply steps per cycle(8000) by cycle multiplier

          //Delay between steps
          timeDelay = commands[3];

          for (int i = 0; i < steps; i++) {
            digitalWrite(STEP_PIN, HIGH);  
            vTaskDelay(timeDelay);     
            digitalWrite(STEP_PIN, LOW);  
            vTaskDelay(timeDelay);       
          }

          digitalWrite(ENABLE_PIN, HIGH);//Turn the stepper motor off

          Serial1.println("2,");//Designated the scan is done to the RPi

          break;
        case 2: //Zero the encoder
          zero_offset = ((shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT) & 0x0FFF) * 360.0) / 4096.0;
          previous_angle = 0;
          turn_count = 0;
          break;
        case 3: //Adjust LED colors
          for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(commands[1], commands[2], commands[3])); // Green
          }
          strip.show();
          break;
        

      }

      Serial.println("Task handled");//Debugging 

      //Clear the commands array after handling the request
      for(int i = 0; i < command_count; i++){
      commands[i]=0;
      }
    }

  }
}

void setup() {
  //Pin setup
  pinMode(DATA_PIN1, INPUT);
  pinMode(CLOCK_PIN1, OUTPUT);
  digitalWrite(CLOCK_PIN1, HIGH);

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  //Always start with the stepper motor off
  digitalWrite(ENABLE_PIN,HIGH);


  Serial.begin(115200);
  delay(50);

  Serial1.setTX(0); // GPIO0
  Serial1.setRX(1); // GPIO1

  // Start UART0
  Serial1.begin(115200);

  delay(50);

  strip.begin();           // Initialize NeoPixel
  strip.show();            // Turn off all LEDs
  strip.setBrightness(255); // Max brightness (optional)
  
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 100, 100)); // cyan for cool looks
  }
  strip.show();

  delay(50);

  // Read encoder once to define zero offset
  int read = shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT);
  zero_offset = ((read & 0x0FFF) * 360.0) / 4096.0;
  previous_angle = 0;

  //All tasks are created here
  xTaskCreate(
    readEncoderTask,
    "EncoderReader",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    requestHandlerTask,
    "RequestHandler",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
  serialCommandTask,
  "SerialCommandReader",
  2048,
  NULL,
  1,
  NULL
  );
}

void loop() {
  // Nothing here; tasks handle everything
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
