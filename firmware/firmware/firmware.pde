#include <Servo.h>

#define MOT_A_DIR 2
#define MOT_A_PWM 1
#define MOT_A_GND 0
#define MOT_B_DIR 7
#define MOT_B_PWM 6
#define MOT_B_GND 5

#define FRAME_SIZE 4

#define ECHO1 24
#define TRIG1 26

#define ECHO2 29
#define TRIG2 30

void setMotors(int8 velA, int8 velB);
uint16 calcPwm(int8 inputVel);
boolean calcDir(int8 inputVel);
volatile unsigned int start1;
volatile unsigned int start2;
volatile float distance1;
volatile float distance2;
unsigned int charCount = 0;
char buf[FRAME_SIZE];

void setup() {
  noInterrupts();
  
  pinMode(ECHO1, INPUT);
  pinMode(TRIG1, OUTPUT);
  
  pinMode(ECHO2, INPUT);
  pinMode(TRIG2, OUTPUT);
  
  start1 = 0;
  start2 = 0;
  
  attachInterrupt(ECHO1, irh1, CHANGE);
  attachInterrupt(ECHO2, irh2, CHANGE);
  
  interrupts();
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(MOT_A_DIR, OUTPUT);
  pinMode(MOT_A_PWM, PWM);
  pinMode(MOT_A_GND, OUTPUT);
  pinMode(MOT_B_DIR, OUTPUT);
  pinMode(MOT_B_PWM, PWM);
  pinMode(MOT_B_GND, OUTPUT);
  
  digitalWrite(MOT_A_GND, LOW);
  digitalWrite(MOT_B_GND, LOW);
  setMotors(0, 0);
}

void loop() {
  while (SerialUSB.available()) {
    
    //SONAR PULSE
    digitalWrite(TRIG1, LOW); 
    delayMicroseconds(2); 

    digitalWrite(TRIG1, HIGH);
    delayMicroseconds(10);
    
    digitalWrite(TRIG1, LOW);
    
    delay(10);
    
    SerialUSB.print("Sonar1: ");
    SerialUSB.println(distance1);
    
    //SONAR PULSE
    digitalWrite(TRIG2, LOW); 
    delayMicroseconds(2); 

    digitalWrite(TRIG2, HIGH);
    delayMicroseconds(10);
    
    digitalWrite(TRIG2, LOW);
    
    delay(10);
    
    SerialUSB.print("Sonar2: ");
    SerialUSB.println(distance2);
    
    //SET MOTORS
    char ch = SerialUSB.read();
    buf[charCount % 4] = ch;
    
    if (charCount == 0 && ch != 'S') {
      continue;
    }
    
    charCount++;
    
    if (ch == 'E') {
      if (charCount == 4) {
        setMotors(buf[1], buf[2]);
      }
      charCount = 0;
      toggleLED();
    }
  }
  
  //SONAR PULSE
  digitalWrite(TRIG1, LOW); 
  delayMicroseconds(2); 

  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
   
  digitalWrite(TRIG1, LOW);
  
  delay(10);
  
  SerialUSB.print("Sonar1: ");
  SerialUSB.println(distance1);
  
  //SONAR PULSE
  digitalWrite(TRIG2, LOW); 
  delayMicroseconds(2); 

  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
   
  digitalWrite(TRIG2, LOW);
  
  delay(10);
  
  SerialUSB.print("Sonar2: ");
  SerialUSB.println(distance2);
  
  delay(100);
}

void irh1(){
  if (digitalRead(ECHO1) == HIGH){
    start1 = micros();
  } else {
    distance1 = (micros() - start1)*0.17;
  }
}

void irh2(){
  if (digitalRead(ECHO2) == HIGH){
    start2 = micros();
  } else {
    distance2 = (micros() - start2)*0.17;
  }
}

void setMotors(int8 velA, int8 velB) {
  digitalWrite(MOT_A_DIR, calcDir(velA));
  pwmWrite(MOT_A_PWM, calcPwm(velA));
  digitalWrite(MOT_B_DIR, calcDir(velB));
  pwmWrite(MOT_B_PWM, calcPwm(velB));
}

uint16 calcPwm(int8 inputVel) {
  uint16 inputVelMag = inputVel > 0 ? inputVel : -inputVel;
  uint16 pwm = (inputVelMag == 128) ? 65535 : inputVelMag << 9;
  return pwm;
}

boolean calcDir(int8 inputVel) {
  return (inputVel > 0);
}
