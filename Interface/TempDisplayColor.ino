// include the library code:
#include <LiquidCrystal.h>
#include <Wire.h>


//*** DESCRIPTION OF PIN CONNECTIONS ***\\
// lcd pin#1 --> ground
// lcd pin#2 --> Vin (%v or 3.3V)
// lcd pin #3 --> potentiometer
// lcd pin#4 --> Arduino digital pin #7
// lcd pin #5 --> ground
// lcd pin #6 --> Arduino digital pin #8
// lcd pin #7 & 8 & 9 & 10 --> empty, no connections
// lcd pin #11 --> Arduino digital pin #9
// lcd pin #12 --> Arduino digital pin #10
// lcd pin #13 --> Arduino digital pin #11
// lcd pin #14 --> Arduino digital pin #12
// lcd pin #15 --> Vin (5V or 3.3V)
// lcd pin #16 --> Arduino digital pin #3
// lcd pin #17 --> Arduino digital pin #5
// lcd pin #18 -->   Arduino digital pin #6


 
#define REDLITE 3
#define GREENLITE 5
#define BLUELITE 6
 
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
 
// you can change the overall brightness by range 0 -> 255
int brightness = 255;
byte celcius[8] = {
  0b11000,
  0b11000,
  0b11,
  0b100,
  0b100,
  0b100,
  0b11
};

byte celcius_symbol[8]{
  0b110,
  0b1001,
  0b1001,
  0b110,
  0b0,
  0b0,
  0b0
  
};


void setup() {
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  lcd.createChar(0,celcius);
  lcd.createChar(1,celcius_symbol);
 
  pinMode(REDLITE, OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE, OUTPUT);
  
 
  brightness = 100;
}

 
void loop() {
  for (int i=0; i<7; i++){
    float temp=35.0+(i*0.5);
    
    if (temp>37.0){
      setBacklight( 255, 0, 0);
      delay(5);
    }
    else if (temp<36.0){
      setBacklight( 0, 0, 255);
      delay(5);
  }
    else{
      setBacklight( 0, 255, 0);
      delay(5);
    }
    
    lcd.setCursor(4,1);
    lcd.print(String(temp, 1));
    lcd.write(byte(1));
    lcd.print("C");
    delay(3000);

    Serial.println(temp);

}
}


void setBacklight(uint8_t r, uint8_t g, uint8_t b) {
 
  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  Serial.print("R = "); Serial.print(r, DEC);
  Serial.print(" G = "); Serial.print(g, DEC);
  Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}
