
int ledPin = 6;

int analogPin = 3; //potentiometer connected to analog pin. Sensing Input.

int val = 0; //variable to store the read value

void setup() {
  // put your setup code here, to run once:
pinMode(ledPin, OUTPUT); //sets the pin as output

Serial.begin(9600); //Open serial port at 100 bps
setPwmFrequency(6,1); // http://playground.arduino.cc/Code/PwmFrequency

}

void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(x), where x is on a scale of 0-255, such that analogWrite(255) requests a 100% duty cycle (always one), and analogWrite(127) is a 50% duty cycle (on half the time) 
  // Syntx: analogWrite(pin, value)

  Serial.print("LED Pin Input:"); 
  Serial.print(val);

  Serial.print("LED Pin Output:");
  Serial.println(val/4);
  val = analogRead(analogPin); //read the intput pin

  analogWrite(ledPin, val/4); //analogRead values go from 0 - 255
}

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
