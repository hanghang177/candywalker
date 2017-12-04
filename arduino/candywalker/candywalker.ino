#include <Servo.h>

#define leftline 9
#define rightline 10
#define servopin 11
#define time360 100

Servo servo;

boolean servorunning = false;
boolean meetleft = false;
boolean meetright = false;

long ptime = millis();

void linecheck(){
  meetleft = !digitalRead(leftline);
  meetright = !digitalRead(rightline);
}

void turn360(){
  servo.writeMicroseconds(2000);
  servorunning = true;
  ptime = millis();
}

void servoend(){
  if(millis() - ptime >= time360){
    servo.writeMicroseconds(1500);
    servorunning = false;
  }
}

void tx(){
  byte txdata = meetleft * 8 + meetright * 4 + servorunning * 2 + 1;
  Serial.write(txdata);
}

void rx(){
  while(Serial.available()){
    if(Serial.parseInt() == 2){
      turn360();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(leftline, INPUT);
  pinMode(rightline, INPUT);
  servo.attach(servopin);
}

void loop() {
  // put your main code here, to run repeatedly:
  linecheck();
  tx();
  rx();
  servoend();
}
