#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define REN 8 // Enable pin for BTS7960
#define LEN 9

#define IN1 5 // IN1 pin for BTS7960
#define IN2 6 // IN2 pin for BTS7960

#define ENCA 3 // Yellow
#define ENCB 4 // White

volatile int posi = 0; // specify posi as volatile
int pos = 0; // declare pos

void setup() {
  Serial.begin(115200); // Initialize serial communication at 9600 baud
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  if (Serial.available()) { // Check if data is available to read
    String input = Serial.readStringUntil('\n'); // Read the incoming string until newline
    
    if (input == "f") {
//      Serial.println("forward");
      digitalWrite(IN1, HIGH); // Rotate motor forward
      digitalWrite(IN2, LOW);
      analogWrite(REN, 255); // Max speed
      analogWrite(LEN, 0); // Disable LEN
    } else if (input == "s") {
//      Serial.println("stop");
      digitalWrite(IN1, LOW); // Stop motor
      digitalWrite(IN2, LOW);
      analogWrite(REN, 0); // Disable REN
      analogWrite(LEN, 0); // Disable LEN
    } else if (input == "b") {
//      Serial.println("backward");
      digitalWrite(IN1, LOW); // Rotate motor backward
      digitalWrite(IN2, HIGH);
      analogWrite(REN, 0); // Disable REN
      analogWrite(LEN, 255); // Max speed
    } else {
//      Serial.println("Invalid input");
    }
  }

//  int a = digitalRead(ENCA);
//  int b = digitalRead(ENCB);
//  Serial.print(a*5); 
//  Serial.print(" ");
//  Serial.print(b*5);
//  Serial.println();

//  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//    pos = posi;
//  }
//  Serial.println(pos);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
