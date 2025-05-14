#include "Servo.h"

const int LMotorEn = 5;
const int LMotor1 = 9;
const int LMotor2 = 10;
const int RMotorEn = 6;
const int RMotor1 = 11;
const int RMotor2 = 12;

const int trigPin = 3;
const int echoPin = 4;

// Create a new servo object:
Servo myservo;
int min = 490;
int max = 2435;
const int servoPin = 8;

double lms = 252;  // left motor's speed (0-255)
double rms = 255;  // right motor's speed (0-255)
double udt = 2500; // time taken to travel unit distance in the maze = 18.5cm (for now)
double t90 = 880;  // turning time for 90 degrees

float distance;
bool turned = false;

void turnRight(int duration) {
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, HIGH);
  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  analogWrite(LMotorEn, lms);
  analogWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void turnLeft(int duration) {
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
  analogWrite(LMotorEn, lms);
  analogWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void moveForward(int duration) {
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, HIGH);
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
  analogWrite(LMotorEn, lms);
  analogWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void moveBackward(int duration) {
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);
  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  analogWrite(LMotorEn, lms);
  analogWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void stopMotors() {
  digitalWrite(LMotorEn, LOW);
  digitalWrite(RMotorEn, LOW);
}

void setup() {
  pinMode(LMotorEn, OUTPUT);
  pinMode(LMotor1,  OUTPUT);
  pinMode(LMotor2,  OUTPUT);
  pinMode(RMotorEn, OUTPUT);
  pinMode(RMotor1,  OUTPUT);
  pinMode(RMotor2,  OUTPUT);

  // Sonar
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo
  myservo.attach(servoPin, min, max);
  myservo.write(87);

  Serial.begin(9600);
}

void loop() {
  // exploreWithWallAvoidance();
  // moveForward(1500);
  // delay(100000);
}

// ----- Task Functions -----

// Task 1: Forward → Stop → Reverse
void taskForwardStopReverse() {
  moveForward(udt);
  delay(3000);             // Stop for 3 seconds
  moveBackward(udt);
}

// Task 2a: Square Path Clockwise
void taskSquareClockwise() {
  for (int i = 0; i < 4; i++) {
    moveForward(udt);
    delay(500);
    turnRight(t90);
    delay(500);
  }
}

// Task 2b: Square Path Anti-Clockwise
void taskSquareAntiClockwise() {
  for (int i = 0; i < 4; i++) {
    moveForward(udt);
    delay(500);
    turnLeft(t90);
    delay(500);
  }
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  distance = pulseIn(echoPin, HIGH) * 0.034 / 2;
  return distance;
}

void moveAndCheckWallStraight() {
  getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 6) {  // obstacle too close
    stopMotors();
    delay(500);
    moveBackward(1000);  // reverse
    turnRight(t90);      // try turning
    turned = true;
  } else {
    moveForward(500);   // safe to move
  }

  delay(100);
}

void moveAndCheckWallMultiDirection() {
  // Face forward
  myservo.write(90);
  delay(300);
  getDistance();
  float frontDist = distance;

  if (frontDist < 7) {
    // Obstacle in front
    stopMotors();
    delay(300);
    moveBackward(1000);
    turnRight(t90);  // try avoiding by turning
    turned = true;
    return;
  }

  // Face 45° to the left
  myservo.write(135);
  delay(300);
  getDistance();
  Serial.print("Left 45° Distance: ");
  Serial.println(distance);

  float leftDist = distance;
  if (leftDist < 7) {
    stopMotors();
    delay(300);
    moveBackward(1000);
    turnRight(t90);  // turn away from the obstacle
    turned = true;
    return;
  }

  // Face 45° to the right
  myservo.write(45);
  delay(300);
  getDistance();
  Serial.print("Right 45° Distance: ");
  Serial.println(distance);

  float rightDist = distance;
  if (rightDist < 7) {
    stopMotors();
    delay(300);
    moveBackward(1000);
    turnLeft(t90);  // turn away from the obstacle
    turned = true;
    return;
  }

  // If no obstacles, move forward
  myservo.write(93);  // center servo
  delay(100);
  moveForward(200);
}

void exploreWithWallAvoidance() {
  while (true) {
    float frontDist = getDistance();
    
    if (frontDist > 3) {
      moveForward(100); // keep moving forward if path is clear
    } else {
      stopMotors();
      delay(200);

      // check left
      turnLeft(t90);
      delay(300);
      float leftDist = getDistance();

      if (leftDist > 7) {
        moveForward(500);
        turned = true;
        return;
      }

      // check right
      turnRight(t90 * 2);  // rotate from left-facing to right-facing
      delay(300);
      float rightDist = getDistance();

      if (rightDist > 7) {
        moveForward(500);
        turned = true;
        return;
      }
    }

    delay(30); // short pause
  }
}


