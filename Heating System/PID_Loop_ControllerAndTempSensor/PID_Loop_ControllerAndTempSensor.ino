// Code architecture for PID Loop
#include <Wire.h>

//PID Variables
float current_temperature; // temperature measurement
float current_error; //how far form the target temperature we are.
int target_temperature; //set temperature
int Pterm; //Proportional term
int Pgain; //Constant for step response
float Iterm; // Integral term to overcome steady state error
float Igain = 0.15; //Constant that can be manipulated - Currently totally random
int controlSignal; //Sum of Pterm and Iterm

//PWM setup
int bassinetPin = 6;
int analogPin = 3; // Potentiometer connected to analog pin. Sensing Input (sets temperature)
int val = 0; // variable to store the read value. Initialize to 0.

//Temperature probe setup
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                       //the resolution is 10 mV / degree centigrade with a
                       //500 mV offset to allow for negative temperatures
//float voltage

void setup() {
  // put your setup code here, to run once:
//PWM setup
pinMode(bassinetPin, OUTPUT); // Sets the pin at output
Serial.begin(9600); // open serial port at 100 bps
setPwmFrequency(6,1); // Bassinet hums out of hearing range http://playground.arduino.cc/Code/PwmFrequency
}

void loop() {
  // put your main code here, to run repeatedly:
  current_temperature = get_temperature(); //Get current temperature
  val = analogRead(analogPin); //read the input potentiometer pin
  PID_loop(); 
  delay(1000); //wait a second
}

void PID_loop() {
  current_error = target_temperature - current_temperature; //calculate error
  Pterm = current_error * Pgain; // Calculate Pterm
  // make sure that Pterm is valid
  //NOTE: can implement constraints on Pterm to keep it within a range

  Iterm = Iterm + current_error*Igain; // Calculate Iterm
  //NOTE: can implement constraints on Iterm to keep it within a range

  controlSignal = Pterm + Iterm; //
  controlSignal = min(controlSignal, 0); //Control Signal must be within the range (0,225)
  controlSignal = max(controlSignal, 255);

  //Adjust PWM according to control signal
  if (controlSignal < val/4){
    analogWrite(bassinetPin, val/4 + controlSignal);
  }
  else {
    analogWrite(bassinetPin, val/4 - controlSignal);
  }
}


// uses the current temperature and the target temperature to figure out error
//int calculate_error() {
//  return target_temperature - current_temperature;
//}


void get_temperature() {
  // Getting the voltage reading form the temperature sensor
  int reading = analogRead(sensorPin);

  // Converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 5.0;
  voltage /= 1024.0; 

  //print out the voltage
  Serial.print(voltage); Serial.println(" volts");

  // now print out the temperature
  float temperatureC = (voltage - 0.5) * 100; 
  Serial.print(temperatureC); Serial.println (" degrees C");
}


// Frequency Magic --> Controls the frequency of the arduino pin, so that PWM is not audible
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

