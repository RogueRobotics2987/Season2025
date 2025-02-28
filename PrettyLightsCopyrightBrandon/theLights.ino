#include <Adafruit_DotStar.h>

// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 144 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    9
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin = 2; // the number of the pushbutton pin
const int buttonPin2 = 4;
const int buttonPin3 = 6;
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
int buttonState3 = 0;
void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  strip.begin();
  strip.clear();
  strip.show(); // Initialize all pixels to 'off'
} 
void CoralInTrough(){
  uint32_t pink = strip.Color(10, 25, 18);
  strip.fill(pink, 0, 144);
}
void CoralInClaw(){
  uint32_t cyan = strip.Color(25, 2, 25);
  strip.fill(cyan, 0, 144); 
}
void LB_BLUE() {
  strip.setPixelColor(0, 0, 0, 50);
}
void RB_RED() {
  strip.setPixelColor(0, 0, 50, 0);
}
void loop() {
    buttonState = digitalRead(buttonPin);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    Serial.println(buttonState);
    Serial.println(buttonState2);
    if (buttonState == 1) {
      CoralInTrough();
      }kjfjfj
    if (buttonState2 == 1) {
      CoralInClaw();
      }
    if (buttonState3 == 1) {
      LB_BLUE();
    }
    if (buttonState2 == 1 && buttonState3 == 1) {
      RB_RED();
    }
    else{
      strip.clear();
    }
    strip.show(); 
}
