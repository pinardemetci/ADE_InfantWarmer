
int bassinetPin = 9;

int analogPin = 3; //potentiometer connected to analog pin. Sensing Input.

int val = 0; //variable to store the read value

void setup() {
  // put your setup code here, to run once:
pinMode(bassinetPin, OUTPUT); //sets the pin as output

Serial.begin(9600); //Open serial port at 100 bps

TCCR2B = TCCR2B & 0b11111000 | 0x01; //sets Arduino Mega's pin 10 and 9 to frequency 31250.
}

void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(x), where x is on a scale of 0-255, such that analogWrite(255) requests a 100% duty cycle (always one), and analogWrite(127) is a 50% duty cycle (on half the time) 
  // Syntx: analogWrite(pin, value)

  Serial.print("Bassinet Pin Output:");
  Serial.println(val/4);
  val = analogRead(analogPin); //read the intput pin

  analogWrite(bassinetPin, val/4); //analogRead values go from 0 - 255
}
