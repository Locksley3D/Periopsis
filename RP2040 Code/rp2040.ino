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

//LED Configuration
#define LED_PIN     5      // GPIO0
#define NUM_LEDS    22    // Total number of WS2812B LEDs

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void readEncoderTask(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10 ms = 100 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency); //This makes the task run at the desired frequency stated above

    int read = shiftIn(DATA_PIN1, CLOCK_PIN1, BIT_COUNT); //Gets the read in bits
    int raw_angle = ((read & 0x0FFF) * 360.0) / 4096.0; //Convert bits to angle data with 12bit resolution

    // Adjust to zero offset captured in setup() and when clearing the encoder
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

    //Send to RPI
    Serial.print("1,");
    Serial.println(int(output_angle*10)); //*10 so no decimals are printed in the Serial bus

  }
}

void serialCommandTask(void *pvParameters) {
  String input = "";

  for (;;) {
    if (Serial.available()) {
      char c = Serial.read();//It may not seem like it but this loops until every character is in the input string ;)

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
          digitalWrite(ENABLE_PIN, LOW); //Turn the stepper motor on

          //Direction
          if(commands[1]==1)digitalWrite(DIR_PIN, LOW);
          else if(commands[1]==2)digitalWrite(DIR_PIN, HIGH); 

          //Half turn of full turn
          if(commands[2]==1)steps = 4000;
          else if(commands[2]==2)steps = 8000;

          //Delay between steps
          timeDelay = commands[3];

          for (int i = 0; i < steps; i++) {
            digitalWrite(STEP_PIN, HIGH);  
            vTaskDelay(timeDelay);     
            digitalWrite(STEP_PIN, LOW);  
            vTaskDelay(timeDelay);       
          }

          digitalWrite(ENABLE_PIN, HIGH);//Turn the stepper motor off
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
