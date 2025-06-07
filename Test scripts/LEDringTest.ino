#include <Adafruit_NeoPixel.h>

// Configuration
#define LED_PIN     5      // GPIO0
#define NUM_LEDS    22    // Total number of WS2812B LEDs

//no 27,28,29,8,7

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();           // Initialize NeoPixel
  strip.show();            // Turn off all LEDs
  strip.setBrightness(255); // Max brightness (optional)
  
  // Set all LEDs to green
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 255)); // Green
  }
  strip.show();
}

void loop() {
  // Nothing to do here
}
