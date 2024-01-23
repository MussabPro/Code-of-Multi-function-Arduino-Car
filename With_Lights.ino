
#include <Servo.h>
#include <AFMotor.h>

#define Echo A0
#define Trig A1
#define motor 10
#define spoint 120
#define flight 12
#define blight 13

int  Speed =200;
char value ='S';
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;
Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
void setup() {
  Serial.begin(9600);
  pinMode(flight, OUTPUT);
  pinMode(blight, OUTPUT);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
 
}
void loop() {
  // Obstacle();
  Bluetoothcontrol();
  // voicecontrol();
}

void Bluetoothcontrol() {  
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  if (value == 'F') {
    forward();
  } else if (value == 'B') {
    backward();
  } else if (value == 'L') {
    left();
  } else if (value == 'R') {
    right();
  } else if (value == 'W') {
    headlightOn();
  } else if (value == 'w') {
    headlightOff();
  } else if (value == 'U') {
    backlightOn();
  } else if (value == 'u') {
    backlightOff();
  }else if (value == 'S') {
    Stop();
  }
}
void Obstacle() {
  distance = ultrasonic();
  
  
  if (distance <= 12) {
    Stop();
    backward();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      right();
      delay(500);
      Stop();
      delay(200);
    } 
    else if (L > R) {
      left();
      delay(500);
      Stop();
      delay(200);
    }
    else if (L == R) {
      backward();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}
void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    if (value == '^') {
      forward();
    } else if (value == '-') {
      backward();
    } else if (value == '<') {
      L = leftsee();
      servo.write(spoint);
      if (L >= 10 ) {
        left();
        delay(500);
        Stop();
      } else if (L < 10) {
        Stop();
      }
    } else if (value == '>') {
      R = rightsee();
      servo.write(spoint);
      if (R >= 10 ) {
        right();
        delay(500);
        Stop();
      } else if (R < 10) {
        Stop();
      }
    } else if (value == '*') {
      Stop();
    }
  }
}
// Ultrasonic sensor distance reading function
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
  
  return cm;
}
void forward() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
  headlightOn();
}
void backward() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
  backlightOn();
}
void right() {
  M1.run(BACKWARD);
  M2.run(BACKWARD);
  M3.run(FORWARD);
  M4.run(FORWARD);
}
void left() {
  M1.run(FORWARD);
  M2.run(FORWARD);
  M3.run(BACKWARD);
  M4.run(BACKWARD);
}
void Stop() {
  M1.run(RELEASE);
  M2.run(RELEASE);
  M3.run(RELEASE);
  M4.run(RELEASE);
  backlightOn();
}
int rightsee() {
  servo.write(60);
  delay(800);
  Left = ultrasonic();
  servo.write(spoint);
  
  if (Left > 12) {
    return Left;
  } else {
    Stop();
    return -1; // Indicate that the distance is less than the threshold
  }
}

int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  servo.write(spoint);
  
  if (Right > 12) {
    return Right;
  } else {
    Stop();
    return -1; // Indicate that the distance is less than the threshold
  }
}
void headlightOn(){
  digitalWrite(flight,HIGH);
  
}
void headlightOff(){
  digitalWrite(flight,LOW);
  
}
void backlightOn(){
  digitalWrite(blight,HIGH);
  
}

void backlightOff(){
  digitalWrite(blight,LOW);
  
}