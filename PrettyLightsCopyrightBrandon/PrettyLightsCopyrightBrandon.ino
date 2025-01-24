#include <Adafruit_DotStar.h>

// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 144 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    9
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
void setup() {
  pinMode(buttonPin, INPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
    //strip.setBrightness(50);
} 
void GBChase () {
  for(int i = 0; i < 144; i++) {
    if (digitalRead(buttonPin) == 0){
      return;
    }
    strip.setPixelColor(i, 25, 0, 0);
    strip.show();
  }
  for(int i = 0; i < 144; i++) {
     if (digitalRead(buttonPin) == 0){
      return;
    }  
    strip.setPixelColor(i, 0, 0, 25);
    strip.show();
  }
}
void loop() {
    uint32_t off = strip.Color(0, 0, 0);
    buttonState = digitalRead(buttonPin);
    if (buttonState == 1) {
     GBChase();
    } else {
      strip.fill(off, 0, 144);
      strip.show();
    }
    //strip.setBrightness(50);
  //GBChase();
 // uint32_t green = strip.Color(50, 0, 0);
  //strip.setPixelColor(0, 255, 0, 0);
  //strip.fill(green, 0, 144);
 
}
