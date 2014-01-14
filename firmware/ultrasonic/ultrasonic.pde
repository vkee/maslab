#include <Servo.h>

#define ECHO 24
#define TRIG 26

volatile unsigned int start;
volatile float distance;

void setup() {
  noInterrupts();
  
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  
  start = 0;
  
  attachInterrupt(ECHO, irh, CHANGE);
  
  interrupts();
}

void loop() {
  digitalWrite(TRIG, LOW); 
  delayMicroseconds(2); 

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(TRIG, LOW);
  
  delay(50);
  SerialUSB.print("Sonar: ");
  SerialUSB.println(distance);
}

void irh(){
  if (digitalRead(ECHO) == HIGH){
    start = micros();
  } else {
    distance = (micros() - start)*0.17;
  }
}
