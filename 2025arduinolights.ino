#include <Adafruit_DotStar.h>

// Because conditional #includes don't work w/Arduino sketches...
#include <SPI.h>         // COMMENT OUT THIS LINE FOR GEMMA OR TRINKET
//#include <avr/power.h> // ENABLE THIS LINE FOR GEMMA OR TRINKET

#define NUMPIXELS 144 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    9
#define CLOCKPIN   8
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
const int buttonPin = 2;// the number of the pushbutton pin
const int buttonPin2 = 3;// the number of the pushbutton pin
const int buttonPin3 = 4;// the number of the pushbutton pin
const int buttonPin4 = 5;// the number of the pushbutton pin


int buttonState = 0;         // variable for reading the pushbutton status
void setup() {
  pinMode(buttonPin, INPUT);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
    //strip.setBrightness(50);
    
} 
void CPickUp () {
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

  void RNDM () {
    int red = random(0, 50);; int green = random(0, 50); int blue = random(0, 50);; 
    int red2 = random(0, 50);; int green2 = random(0, 50);; int blue2 = random(0, 50); 
     int red3 = random(0, 50);; int green3 = random(0, 50);; int blue3 = random(0, 50);;
      int red4 = random(0, 50); int green4 = random(0, 50); int blue4 = random(0, 50);; 
      int red5 = random(0, 50);; int green5 = random(0, 50); int blue5 = random(0, 50);; 
    int red6 = random(0, 50); int green6 = random(0, 50);; int blue6 = random(0, 50);; 
     int red7 = random(0, 50); int green7 = random(0, 50); int blue7 = random(0, 50);
      int red8 = random(0, 50);; int green8 = random(0, 50);; int blue8 = random(0,30); 
      int red9 = random(0, 50); int green9 = random(0, 50);; int blue9 = random(0,50); 
    int red10 = random(0, 50);; int green10 = random(0, 50);; int blue10 = random(0, 50); 
     int red11 = random(0, 50); int green11 = random(0, 50); int blue11 = random(0, 50);
      int red12 = random(0, 50);; int green12 = random(0, 50);; int blue12 = random(0, 50); 
  for(int i = 0; i < 12; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red, green, blue);
    strip.show();
  }
  for(int i = 12; i < 24; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }  
    strip.setPixelColor(i, red2, green2, blue2);
    strip.show();
  }
   for(int i = 24; i < 36; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red3, green3, blue3);
    strip.show();
  }
  for(int i = 36; i < 48; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red4, green4, blue4);
    strip.show();
    }
    for(int i = 48; i < 60; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red5, green5, blue5);
    strip.show();
  }
  for(int i = 60; i < 72; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }  
    strip.setPixelColor(i, red6, green6, blue6);
    strip.show();
  }
   for(int i = 72; i < 84; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red7, green7, blue7);
    strip.show();
  }
  for(int i = 84; i < 96; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red8, green8, blue8);
    strip.show();
    }
    for(int i = 96; i < 108; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red9, green9, blue9);
    strip.show();
  }
  for(int i = 108; i < 120; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }  
    strip.setPixelColor(i, red10, green10, blue10);
    strip.show();
  }
   for(int i = 120; i < 132; i++) {
    if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red11, green11, blue11);
    strip.show();
  }
  for(int i = 132; i < 144; i++) {
     if (digitalRead(buttonPin2) == 0){
      return;
    }
    strip.setPixelColor(i, red12, green12, blue12);
    strip.show();
    }
  }
  void goVIKES(){
    uint32_t Yellow = strip.Color(25, 25, 0);
    uint32_t Purple = strip.Color(0, 25, 25);
  
    if (digitalRead(buttonPin3) == 0){
      return;
    }
   strip.fill(Yellow, 0, 6); strip.fill(Purple, 6, 6);   strip.fill(Yellow, 12, 6);   strip.fill(Purple, 18, 6);   strip.fill(Yellow, 24, 6);   strip.fill(Purple, 30, 6);   strip.fill(Yellow, 36, 6);   strip.fill(Purple, 42, 6);
   strip.fill(Yellow, 48, 6);   strip.fill(Purple, 54, 6);   strip.fill(Yellow, 60, 6);   strip.fill(Purple, 66, 6);   strip.fill(Yellow, 72, 6);   strip.fill(Purple, 78, 6);   strip.fill(Yellow, 84, 6);   strip.fill(Purple, 90, 6);
   strip.fill(Yellow, 96, 6);   strip.fill(Purple, 102, 6);   strip.fill(Yellow, 108, 6);   strip.fill(Purple, 114, 6);   strip.fill(Yellow, 120, 6);   strip.fill(Purple, 126, 6);   strip.fill(Yellow, 132, 6);
   strip.fill(Purple, 138, 6);   strip.fill(Yellow, 144, 6);
   strip.show();
  
  delay(500);
     
   strip.fill(Purple, 0, 6); strip.fill(Yellow, 6, 6);   strip.fill(Purple, 12, 6);   strip.fill(Yellow, 18, 6);   strip.fill(Purple, 24, 6);   strip.fill(Yellow, 30, 6);   strip.fill(Purple, 36, 6);   strip.fill(Yellow, 42, 6);
   strip.fill(Purple, 48, 6);   strip.fill(Yellow, 54, 6);   strip.fill(Purple, 60, 6);   strip.fill(Yellow, 66, 6);   strip.fill(Purple, 72, 6);   strip.fill(Yellow, 78, 6);   strip.fill(Purple, 84, 6);   strip.fill(Yellow, 90, 6);
   strip.fill(Purple, 96, 6);   strip.fill(Yellow, 102, 6);   strip.fill(Purple, 108, 6);   strip.fill(Yellow, 114, 6);   strip.fill(Purple, 120, 6);   strip.fill(Yellow, 126, 6);   strip.fill(Purple, 132, 6);
   strip.fill(Yellow, 138, 6);   strip.fill(Purple, 144, 6);
   strip.show();
  delay(500);
  
  }void APickUp(){

  for(int i = 0; i < 72; i++){
    for(int j = 144; j > 72; j--){
     
  
    if(digitalRead(buttonPin4) == 0){
      return;
    }
    strip.setPixelColor(i, 0, 45, 0);
    strip.show();
  }
  }
  delay(1000);
  for(int i = 72; i > 0; i++){
    for(int j = 72; i < 144; i--){
      if(digitalRead(buttonPin4) == 0){
        return;
      }
       strip.setPixelColor(j, 15, 15, 15);
       strip.show(); 
    }
  }
}

void idleRR () {
  for(int i = 0; i < 48; i++){
    if(digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
    }
  for(int i = 48; i < 96; i++){
    if(digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 25, 25, 25);
    strip.show();
  }
    for(int i = 96; i < 144; i++){
    if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
    }
  for(int i = 0; i < 48; i++){
    if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 25, 25, 25);
    strip.show();
  }
  for(int i = 48; i < 96; i++){
    if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 0, 25, 0);
    strip.show();
    }
  for(int i = 96; i < 144; i++){
    if (digitalRead(buttonPin) == 1 || digitalRead(buttonPin2) == 1 || digitalRead(buttonPin3) == 1 || digitalRead(buttonPin4) == 1){
      return;
    }
    strip.setPixelColor(i, 25, 25, 25);
    strip.show();
  }
}

void loop() {
//    uint32_t off = strip.Color(0, 0, 0); 
    buttonState = digitalRead(buttonPin);
    if (buttonState == 1) {
     CPickUp();
    } 
    else {     
      idleRR();
    }
    buttonState = digitalRead(buttonPin2);
    if (buttonState == 1) {
       RNDM();
    }
    else {
      idleRR();
    }
    buttonState = digitalRead(buttonPin3);
    if (buttonState == 1) {
      goVIKES();
    }
    else{
      idleRR();
    }
//  c
    //strip.setBrightness(50);
  //GBChase();
 // uint32_t green = strip.Color(50, 0, 0);
  //strip.setPixelColor(0, 2100, 0, 0);
  //strip.fill(green, 0, 144);
 
}
