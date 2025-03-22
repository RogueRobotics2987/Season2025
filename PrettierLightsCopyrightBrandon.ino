#include <Adafruit_DotStar.h>

// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 144 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    9
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin = 0; // the number of the pushbutton pin
const int buttonPin2 = 2;
const int buttonPin3 = 4;
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
int buttonState3 = 0;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  strip.begin();
  strip.clear();
  strip.show();
}

void Idle () {
  for(int i = 0; i < 144; i++) {
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
  }
   for(int i = 0; i < 144; i++) {
    strip.setPixelColor(i, 0, 0, 25);
    strip.show();
  }
}

void GreenBlink() { //make it blink or fade or something
  uint32_t green = strip.Color(50, 0, 0);
  strip.fill(green, 0, 144);
  strip.show();
  delay(250);
  strip.clear();
  delay(250);
}

void Green() {
  uint32_t green = strip.Color(50, 0, 0);
  strip.fill(green, 0, 144);
  strip.show();
}

void Red() {
  uint32_t red = strip.Color(0, 0, 50);
  strip.fill(red, 0, 144);
  strip.show();
}

void RedBlink() { //make it blink or fade or something
  uint32_t red = strip.Color(0, 0, 50);
  strip.fill(red, 0, 144);
  strip.show();
    delay(250);
  strip.clear();
  delay(250);
}

void Blue() {
  uint32_t blue = strip.Color(0, 50, 0);
  strip.fill(blue, 0, 144);
  strip.show();
}

void BlueBlink() { //make it blink or fade or something
  uint32_t blue = strip.Color(0, 50, 0);
  strip.fill(blue, 0, 144);
  strip.show();
    delay(250);
  strip.clear();
  delay(250);
}

void loop() {
  buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);

  if (buttonState == 1 && buttonState2 == 0 && buttonState3 == 0) {
    GreenBlink();
  }
  else if (buttonState == 0 && buttonState2 == 1 && buttonState3 == 0) {
    Green();
  }
  else if (buttonState == 0 && buttonState2 == 0 && buttonState3 == 1) {
    RedBlink();
  }
  else if (buttonState == 1 && buttonState2 == 1 && buttonState3 == 0) {
    Red();
  }
  else if (buttonState == 1 && buttonState2 == 0 && buttonState3 == 1) {
    BlueBlink();
  }
  else if (buttonState == 0 && buttonState2 == 1 && buttonState3 == 1) {
    Blue();
  }
  else {
    Idle();
  }
}
