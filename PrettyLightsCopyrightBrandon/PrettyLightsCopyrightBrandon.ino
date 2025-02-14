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
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
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
void loop() {
    buttonState = digitalRead(buttonPin);
    buttonState2 = digitalRead(buttonPin2);
    Serial.println(buttonState);
    Serial.println(buttonState2);
    if (buttonState == 1) {
      CoralInTrough();
      Serial.println("coral");
    }
    else if (buttonState2 == 1) {
      CoralInClaw();
      Serial.println("claw");
    }
    else{
      strip.clear();
      Serial.println("clear");
    }
    strip.show(); 
}
