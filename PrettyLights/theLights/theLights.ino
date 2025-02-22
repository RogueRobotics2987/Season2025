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
  pinMode(buttonPin3, INPUT);
  strip.begin();
  strip.clear();
  strip.show(); // Initialize all pixels to 'off'
  Serial.begin(4800);
} 

void CoralManipulator(){
    uint32_t Red = strip.Color(0, 50, 0);
    uint32_t Blue = strip.Color(0, 0, 50);
  
    strip.fill(Red, 0, 6); strip.fill(Blue, 6, 6);   strip.fill(Red, 12, 6);   strip.fill(Blue, 18, 6);   strip.fill(Red, 24, 6);   strip.fill(Blue, 30, 6);   strip.fill(Red, 36, 6);   strip.fill(Blue, 42, 6);
    strip.fill(Red, 48, 6);   strip.fill(Blue, 54, 6);   strip.fill(Red, 60, 6);   strip.fill(Blue, 66, 6);   strip.fill(Red, 72, 6);   strip.fill(Blue, 78, 6);   strip.fill(Red, 84, 6);   strip.fill(Blue, 90, 6);
    strip.fill(Red, 96, 6);   strip.fill(Blue, 102, 6);   strip.fill(Red, 108, 6);   strip.fill(Blue, 114, 6);   strip.fill(Red, 120, 6);   strip.fill(Blue, 126, 6);   strip.fill(Red, 132, 6);
    strip.fill(Blue, 138, 6);   strip.fill(Red, 144, 6);
    strip.show();
    delay(250);
    
    strip.fill(Blue, 0, 6); strip.fill(Red, 6, 6);   strip.fill(Blue, 12, 6);   strip.fill(Red, 18, 6);   strip.fill(Blue, 24, 6);   strip.fill(Red, 30, 6);   strip.fill(Blue, 36, 6);   strip.fill(Red, 42, 6);
    strip.fill(Blue, 48, 6);   strip.fill(Red, 54, 6);   strip.fill(Blue, 60, 6);   strip.fill(Red, 66, 6);   strip.fill(Blue, 72, 6);   strip.fill(Red, 78, 6);   strip.fill(Blue, 84, 6);   strip.fill(Red, 90, 6);
    strip.fill(Blue, 96, 6);   strip.fill(Red, 102, 6);   strip.fill(Blue, 108, 6);   strip.fill(Red, 114, 6);   strip.fill(Blue, 120, 6);   strip.fill(Red, 126, 6);   strip.fill(Blue, 132, 6);
    strip.fill(Red, 138, 6);   strip.fill(Blue, 144, 6);
    strip.show();
    delay(250);
}

void RFRCoralFind(){
  uint32_t pink = strip.Color(7, 25, 18);
  strip.fill(pink, 0, 144);
}

void RFRCoralPlace(){
  uint32_t pink = strip.Color(7, 25, 18);   
  uint32_t off = strip.Color(0, 0, 0);
  strip.fill(pink, 0, 144);
  strip.show();
  delay(100);
  strip.fill(off, 0, 144);
  strip.show();
  delay(100);
}

void LFRCoralFind(){
  uint32_t cyan = strip.Color(26, 0, 26);
  strip.fill(cyan, 0, 144); 
}

void LFRCoralPlace(){
  uint32_t cyan = strip.Color(26, 0, 26);
  uint32_t off = strip.Color(0, 0, 0);
  strip.fill(cyan, 0, 144);
  strip.show();
  delay(100);
  strip.fill(off, 0, 144);
  strip.show();
  delay(100);
}
  
void loop() {
    buttonState = digitalRead(buttonPin);
    buttonState2 = digitalRead(buttonPin2);
    buttonState3 = digitalRead(buttonPin3);
    Serial.println(buttonState);
    Serial.println(buttonState2);
    Serial.println(buttonState3);    

    if(buttonState == 1 && buttonState2 == 1) {
      RFRCoralPlace();
        Serial.println("RFRblink");
    }
    else if(buttonState == 1 && buttonState3 == 1){
      LFRCoralPlace();
        Serial.println("LFRblink");
    }
    else if (buttonState == 1) {
      CoralManipulator();
        Serial.println("Manipulator");
    }
    else if (buttonState2 == 1) {
      RFRCoralFind();
        Serial.println("RFRstay");
    }
    else if (buttonState3 == 1) {
      LFRCoralFind();
        Serial.println("LFRstay");
    }

        else{
      strip.clear();
       Serial.println("clear");
    }

    strip.show(); 
}
