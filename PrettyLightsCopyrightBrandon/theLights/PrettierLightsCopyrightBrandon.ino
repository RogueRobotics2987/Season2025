#include <Adafruit_DotStar.h>

// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 116 // Number of LEDs in strip 

// Here's how to control the LEDs from any two pins:
#define DATAPIN    8
#define CLOCKPIN   9
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin = 4; // the number of the pushbutton pin
const int buttonPin2 = 2;
const int buttonPin3 = 6;
int buttonState = digitalRead(buttonPin);         // variable for reading the pushbutton status
int buttonState2 = digitalRead(buttonPin2);
int buttonState3 = digitalRead(buttonPin3);

//void brighten() {
//  for (int i = 0; i > 100; i++) {
//    strip.setBrightness(i);
//    strip.show();
//  }
//}
//
//void darken() {
//  for (int i = 100; i > 0; i--) {
//    strip.setBrightness(i);
//    strip.show();
//  }
//}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
//  Serial.begin(9600);
  strip.begin();
  strip.clear();
  strip.show();
}

void Off() {
  strip.clear();
  strip.show();
}

void Idle () {
  for(int i = 0; i < NUMPIXELS; i++) {
    buttonState = digitalRead(buttonPin); 
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
//    Serial.println("Idle");
    if (!(buttonState == 1 && buttonState2 == 1 && buttonState3 == 1)) {
      return;
    } 
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
  }
   for(int i = 0; i < NUMPIXELS; i++) {
    buttonState = digitalRead(buttonPin); 
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    if (!(buttonState == 1 && buttonState2 == 1 && buttonState3 == 1)) {
      return;
    } 
    strip.setPixelColor(i, 0, 0, 25);
    strip.show();
  }
}

void GreenBlink() {
  uint32_t green = strip.Color(50, 0, 0);
  uint32_t off = strip.Color(0, 0, 0);
  strip.fill(green , 0, NUMPIXELS);
  delay(100);
  buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  if (!(buttonState == 1 && buttonState2 == 0 && buttonState3 == 0)) {
    return;
  } 
  strip.show();
  strip.fill(off , 0, NUMPIXELS);
  delay(100);
  strip.show();
}

void Green() {
  uint32_t green = strip.Color(50, 0, 0);
  strip.fill(green, 0, NUMPIXELS);
  strip.show();
}

void Red() {
  uint32_t red = strip.Color(0, 50, 0);
  strip.fill(red, 0, NUMPIXELS);
  strip.show();
  Serial.println("Red");
}

void Orange() {
  uint32_t orange = strip.Color(5, 50, 0);
  strip.fill(orange, 0, NUMPIXELS);
  strip.show();
}

void RedBlink() {
  uint32_t red = strip.Color(0, 50, 0);
  uint32_t off = strip.Color(0, 0, 0);
  strip.fill(red , 0, NUMPIXELS);
  delay(100);
  buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  if (!(buttonState == 0 && buttonState2 == 0 && buttonState3 == 1)) {
    return;
  } 
  strip.show();
  strip.fill(off , 0, NUMPIXELS);
  delay(100);
  strip.show();
}

void Blue() {
  uint32_t blue = strip.Color(0, 0, 50);
  strip.fill(blue, 0, NUMPIXELS);
  strip.show();
}

void BlueBlink() {
  uint32_t blue = strip.Color(0, 0, 50);
  uint32_t off = strip.Color(0, 0, 0);
  strip.fill(blue , 0, NUMPIXELS);
  delay(100);
  buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  if (!(buttonState == 1 && buttonState2 == 0 && buttonState3 == 1)) {
    return;
  } 
  strip.show();
  strip.fill(off , 0, NUMPIXELS);
  delay(100);
  strip.show();
}

void loop() {
  buttonState = digitalRead(buttonPin); 
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  Serial.print(buttonState);
  Serial.print(buttonState2);
  Serial.println(buttonState3);
  
  if (buttonState == 0 && buttonState2 == 0 && buttonState3 == 0) {
    Off();
  }
 else if (buttonState == 1 && buttonState2 == 0 && buttonState3 == 0) {
    GreenBlink();
  }
  else if (buttonState == 0 && buttonState2 == 1 && buttonState3 == 0) {
    Green();
  }
  else if (buttonState == 0 && buttonState2 == 0 && buttonState3 == 1) {
    Orange();
  }
  else if (buttonState == 1 && buttonState2 == 1 && buttonState3 == 0) {
    Red();
  }
  else if (buttonState == 1 && buttonState2 == 0 && buttonState3 == 1) {
    Orange();
  }
  else if (buttonState == 0 && buttonState2 == 1 && buttonState3 == 1) {
    Blue();
  }
  else {
    Idle();
  }
}