#include <Adafruit_DotStar.h>
#include <SPI.h>

#define NUMPIXELS 144 // Number of LEDs in strip

#define DATAPIN    9
#define CLOCKPIN   8

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin2 = 5;
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
void setup() {
    pinMode(buttonPin, INPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

}
 void Algae () {  
  uint32_t green = strip.Color(50, 0, 0);
  for(int i = 0; i < 144; i++) {
  if (digitalRead(buttonPin) == 0){
    return;
    }
    strip.fill(green, 0, 144);
    strip.show();
  }

}

void Coral () {
  uint32_t Yellow = strip.Color(25, 25, 0);
  for(int i = 0; i < 144; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.fill(Yellow, 0, 144);
    strip.show();
  }
}



void RBChase () {
  for(int i = 0; i < 144; i++) {
     if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1) {
      return;
     }
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
  }
   for(int i = 0; i < 144; i++) {
    if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1){
      return;
     }
    strip.setPixelColor(i, 0, 0, 25);
    strip.show();
  }
}
void loop() {
   uint32_t off = strip.Color(0, 0, 0);
   uint32_t green = strip.Color(50, 0, 0);
   buttonState = digitalRead(buttonPin);
   if (buttonState == 1) {
     Algae();
    } else {
      RBChase();
    }
    buttonState = digitalRead(buttonPin2);
    if(buttonState == 1) {
      Coral();
    } else {
      RBChase();
    }
}
