// Code architecture for PID Loop
#include <Wire.h>

float current_temperature; // temperature measurement
int current_error; //how far form the target temperature we are.
int target_temperature; //set temperature
int Pterm; //Proportional term
int Pgain; //Constant for step response
int I_max = 200;
int I_min = -200;
float Iterm; // Integral term to overcome steady state error
float Igain = 0.15; //Constant that can be manipulated
int controlSignal; //Sum of Pterm and Iterm


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  PID_loop();
  delay(10); //100 Hz
}

void PID_loop() {
  current_error = calculate_error(); //calculate error
  Pterm = current_error * Pgain; // Calculate Pterm
  // make sure that Pterm is valid
  //NOTE: can implement constraints on Pterm to keep it within a range

  Iterm = Iterm + current_error*Igain; // Calculate Iterm
  //NOTE: can implement constraints on Iterm to keep it within a range

  controlSignal= Pterm + Iterm; //
}

// uses the current temperature and the target temperature to figure out error
int calculate_error() {
  return target_temperature - current_temperature;
}

