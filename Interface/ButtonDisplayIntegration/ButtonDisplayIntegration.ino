//****** FOR CONTROLING TEMPERATURE DISPLAY WITH UP/DOWN BUTTONS *****\\

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
// lcd pin #18 -->   Arduino digital pin #7


/////////FOR UP/DOWN BUTTONS:
// Button pin 1 --> ground
// button pin 2 --> Arduino digital 4
// button pin 3 --> Arduino digital 1

#include <LiquidCrystal.h>
#include <Wire.h>

const int buttonPin[]={4, 13, 0, 1};
int buttonState=-1;
float temp = 36.0;
String State= "NORMAL";
String lastState="NORMAL";
String reading="NORMAL";
unsigned long lastDebounceTime=0;
unsigned long debounceDelay= 50;

#define REDLITE 3
#define GREENLITE 5
#define BLUELITE 6
LiquidCrystal lcd(7,8,9,10,11,12);

int brightness=255;
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
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int x=0; x<2; x++){
    pinMode (buttonPin[x], INPUT_PULLUP);
  }
lcd.begin(16, 2);
  lcd.createChar(0,celcius);
  lcd.createChar(1,celcius_symbol);
 
  pinMode(REDLITE, OUTPUT);
  pinMode(GREENLITE, OUTPUT);
  pinMode(BLUELITE, OUTPUT);
  
 
  brightness = 100;
}

void loop() {
  // put your main code here, to run repeatedly:
   for(int x=0; x<2; x++)
  {
    //signifying the state of which the button is in by reading the appropriate pin #
    buttonState = digitalRead(buttonPin[x]);

    // check if the pushbutton on the keypad is pressed.
    // if it is, the buttonState is LOW:
    if (buttonState == LOW && buttonPin[x] == 4) {
      float debouncer = millis();
      if (debouncer > 1000){          
      temp=temp+0.5;
      delay(400);
            }
      }
    
    if (buttonState == LOW && buttonPin[x] == 13) {
      // turn LED off:
       float debouncer = millis();
      if (debouncer > 1000){          
      temp=temp-0.5;
      delay(400);
      }

    }}
    lcd.setCursor(0,1);
    lcd.print(String(35.0, 1));
    lcd.write(byte(1));
    lcd.print("C");

    lcd.setCursor(10,1);
    lcd.print(String(temp, 1));
    lcd.write(byte(1));
    lcd.print("C");


    // Currently the backlight color is controlled by the set temperature but we want to change this to bassinet temperature reading
    if (temp>37.0){
      setBacklight( 255, 0, 0);
     }
    else if (temp<36.0){
      setBacklight( 0, 0, 255);
  }
    else{
      setBacklight( 0, 255, 0);
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
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}   









  


