#include <Adafruit_NeoPixel.h>

#define PIN 6

Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, PIN, NEO_GRBW + NEO_KHZ800);

int whiteValue = 0;
int redValue = 255;
float redValueFaded;
int greenValue = 0;
float greenValueFaded;
int blueValue = 0;

float fader = 0;

long time;

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {

  time = millis();

  determineLedTemperature();
  strip.show();
//  delay(50);
//  Serial.println(time);

}

void determineLedTemperature() {
  // read temperature from processing (or other arduino)
  // ledTemperature = ... incomming data;

  greenValue = map(/* ledTemperature */24, 10, 30, 200, 0);
  
  for (int i = 0; i < strip.numPixels(); i++) {
      determineLedBrightness();
      greenValueFaded = greenValue * fader;
      redValueFaded = redValue * fader;
      strip.setPixelColor(i, redValueFaded, greenValueFaded, blueValue, whiteValue);
  }
  strip.show();
//  delay(wait);
}

void determineLedBrightness() {
  // get the amount of radiators that are turned on from processing

  fader = (0.5) * sin((float(time)/1000) - (PI*0.5)) + (0.5);
  Serial.println(fader);

//   fader = (1/2) * sin((time/1000) /* * amountTurnedOn*/ - (PI/2)) + (1/2);

}

