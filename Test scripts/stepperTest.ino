// Pin Definitions
#define ENABLE_PIN 27
#define STEP_PIN 28
#define DIR_PIN 29

void setup() {
  // Set up the pins
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Enable the motor driver (active low)
  digitalWrite(ENABLE_PIN, LOW);

  // Set the direction (1 for one direction, 0 for the other)
  digitalWrite(DIR_PIN, LOW);  // Change to LOW to reverse direction
}

void loop() {
  // Spin the motor 1000 steps with a short delay between each step
  for (int i = 0; i < 1000; i++) {
    digitalWrite(STEP_PIN, HIGH);  // Send a pulse to the step pin
    delayMicroseconds(1000);        // Adjust delay to control motor speed
    digitalWrite(STEP_PIN, LOW);   // End the pulse
    delayMicroseconds(1000);        // Adjust delay to control motor speed
  }

  // Optionally, add a pause between steps or to change direction
  delay(1000);  // 1-second pause after completing the steps
}
