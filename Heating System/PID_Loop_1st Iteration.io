#include <DallasTemperature.h>

// Code architecture for PID Loop
// Uses waterproof sensor

#include <Wire.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


//PID Variables
float current_temperature; // temperature measurement
float current_error; //how far form the target temperature we are.
float target_temperature = 30.00; //set temperature
float Pterm; //PropoMrtional term
float Pgain = 1.5; //Constant for step response
float Iterm; // Integral term to overcome steady state error
float Igain = 0.1; //Constant that can be manipulated - Currently totally random
int controlSignal; //Sum of Pterm and Iterm
int allowable_temp_range; // temperature tolerance

//PWM setup
int bassinetPin = 6;
int analogPin = 3; // Potentiometer connected to analog pin. Sensing Input (sets temperature)
float val = 0; // variable to store the read value. Initialize to 0.
float dutyCycle = 0.0; //Duty Cycle of Bassinet

//Temperature probe setup
//int sensorPin = 1; //the analog pin the TMP36's Vout (sense) pin is connected to
                       //the resolution is 10 mV / degree centigrade with a
                       //500 mV offset to allow for negative temperatures

void setup() {
  // put your setup code here, to run once:
  //PWM setup
  pinMode(bassinetPin, OUTPUT); // Sets the pin at output
  Serial.begin(9600); // open serial port at 100 bps
  setPwmFrequency(6,1); // Bassinet hums out of hearing range http://playground.arduino.cc/Code/PwmFrequency
  sensors.begin();
//  allowable_temp_range = 5;
}

void loop() {
  // put your main code here, to run repeatedly:
  current_temperature = get_temperature(); //Get current temperature
  val = analogRead(analogPin)/4.0; //read the input potentiometer pin
  dutyCycle = (val/255.0)*100.0; //Calculate the Duty Cycle
//  Serial.print("Potentiometer Setting: "); Serial.println(val);
//  Serial.print("Duty Cycle: "); Serial.println(dutyCycle);  
//  analogWrite(bassinetPin, val);
//  target_temperature = get_target_temperature();
  PID_loop(); 
  delay(1000); //wait a second

//  raise_alarm();
}

void PID_loop() {
  current_error = target_temperature - current_temperature; //calculate error
  Serial.print("Current Error: "); Serial.println(current_error);
  Pterm = current_error * Pgain; // Calculate Pterm
  Serial.print("Pterm: "); Serial.println(Pterm); 
  
  // make sure that Pterm is valid
  //NOTE: can implement constraints on Pterm to keep it within a range

  Iterm = Iterm + current_error*Igain; // Calculate Iterm
  Serial.print("Iterm: "); Serial.println(Iterm);
  //NOTE: can implement constraints on Iterm to keep it within a range

  controlSignal = Pterm + Iterm; //
  if (controlSignal < 0){
    Iterm = 0;
    controlSignal = 0; 
  }
  if (controlSignal > 255) {
    Iterm = 0;
    controlSignal = 255;
  }
  Serial.print("Control Singal: "); Serial.println(controlSignal); 
  analogWrite(bassinetPin, controlSignal);
  Serial.print("Output PWM: "); Serial.println(controlSignal); 
}

  //Adjust PWM according to control signal
//  if (current_error > 0){
//    analogWrite(bassinetPin, controlSignal);
//    Serial.print("Output PWM for Heating: "); Serial.println(controlSignal); 
//  }
//  else {
//    analogWrite(bassinetPin, controlSignal);
//    Serial.print("Output PWM for Cooling: "); Serial.println(controlSignal); 
//  }
//  
//  if (controlSignal < val/4){
//    
//    analogWrite(bassinetPin, val/4 + controlSignal);
//    Serial.print("Output PWM for Heating: "); Serial.println(val/4 + controlSignal); 
//  }
//  else {
//    analogWrite(bassinetPin, val/4 - controlSignal);
//    Serial.print("Output PWM for Cooling: "); Serial.println(val/4 - controlSignal); 
//  }
//}


int get_temperature() {

  sensors.requestTemperatures();
  Serial.print("Temperature is: " );
  Serial.println(sensors.getTempCByIndex(0));
  return sensors.getTempCByIndex(0);
 
// !!!!!!! THIS CODE IS FOR THE DIGITAL SENSOR !!!!!!! \\\\\\\
// READ DATA
//Serial.println("DHT11, \t");
//int chk = DHT.read11(DHT1_PIN);
//switch (chk)
//{
// case DHTLIB_OK:  
// Serial.println("OK,\t"); 
// break;
// case DHTLIB_ERROR_CHECKSUM: 
// Serial.println("Checksum error,\t"); 
// break;
// case DHTLIB_ERROR_TIMEOUT: 
// Serial.println("Time out error,\t"); 
// break;
// default: 
//Serial.println("Unknown error,\t"); 
//  break;
//}



  
// DISPLAY DATA
//  Serial.print(DHT.humidity,1);
//  Serial.print(",\t");
//  Serial.println(DHT.temperature,1);
//
//  delay(1000);
//  return DHT.temperature;
// END OF CODE FOR DIGITAL SENSOR


  
////*** !!!!! THIS CODE IS FOR ANALOG SENSOR !!!! **** \\\\\\
////getting the voltage reading from the temperature sensor
//int reading1 = analogRead(sensorPin1); 
//// converting that reading to voltage, for 3.3v arduino use 3.3
// float voltage1 = (reading1 * 5.0)/1024.0;
//// now print out the temperature
// float temperature1C = (voltage1 - 0.5) * 100 ; 
// Serial.print(temperature1C); Serial.println(" degrees C");
// delay(1000);
// return temperature1C;
// END OF CODE FOR ANALOG  
}


int get_target_temperature() {
  return (9/41)*(val/4);
  }

//void raise_alarm() {
///**
// * If temperature of bassinet is out-of-range, raise an alarm.
// */
//  if (current_temperature >= 38) {
//    Serial.println("Temp too HIGH; outside allowable range");
//  };
//  if (current_temperature <= 28) {
//    Serial.println("Temp too LOW; outside allowable range");
//  };
//}

// uses the current temperature and the target temperature to figure out error
//int calculate_error() {
//  return target_temperature - current_temperature;
//}


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
