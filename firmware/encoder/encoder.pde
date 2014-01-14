#define PINA 8
#define PINB 9

long count = 0;

void setup(){
  noInterrupts();
  
  pinMode(PINA, INPUT);
  pinMode(PINB, INPUT);
  
  attachInterrupt(PINA, irh, RISING);
  
  interrupts();
}

void loop(){
  if (micros() % 50000 == 0){
    SerialUSB.println(count);
  }
}

void irh(){
  if (digitalRead(PINB) == HIGH){
    count++;
  } else {
    count--;
  }
}
