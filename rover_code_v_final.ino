#include "Servo.h"
#include <math.h>

const float sonar_separation = 3.2;// Distance between the sonar when turning left-most and right-most
const float wall_distance = 18.0;  // Maze wall spacing in cm
const int NUM_SAMPLES = 5;         // Number of sonar measurements
const float WALL_THRESHOLD = 15;   // Maximum distance of wall detection

const int LMotorEn = 5;
const int LMotor1 = 9;
const int LMotor2 = 10;
const int RMotorEn = 6;
const int RMotor1 = 11;
const int RMotor2 = 12;

const int trigPin = 3;
const int echoPin = 4;

Servo myservo;
int min = 485;
int max = 2435;
const int servoPin = 8;

double lms = 255;    // left motor's speed (0-255)
double rms = 255;    // right motor's speed (0-255)
double udt = 2730;   // time taken to travel unit distance in the maze = 18.5cm 
double t90l = 1100;  // turning time for 90 degrees
double t90r = 950;   // turning time for 90 degrees

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

void stopMotors() {
  digitalWrite(LMotorEn, LOW);
  digitalWrite(RMotorEn, LOW);
}

void turnServoLeft90() {
  myservo.write(180);
  delay(200);
} 

void turnServoStraight() {
  myservo.write(87);
  delay(200);
}

void turnServoRight90() {
  myservo.write(0);
  delay(200);
} 

float trigger_and_record() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH, 20000); // timeout after 20ms
  return duration * 0.034 / 2 - 0.4; // Convert to cm
}

void eliminateOutliers(float* arr, int& size, float threshold = 1.0) {
  // Step 1: Calculate mean
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  float mean = sum / size;

  // Step 2: Calculate standard deviation
  float variance = 0;
  for (int i = 0; i < size; i++) {
    variance += (arr[i] - mean) * (arr[i] - mean);
  }
  float stddev = sqrt(variance / size);

  // Step 3: Filter elements based on Z-score
  int newSize = 0;
  for (int i = 0; i < size; i++) {
    float z = (arr[i] - mean) / stddev;
    if (fabs(z) <= threshold) {
      arr[newSize++] = arr[i];
    }
  }

  size = newSize;
}

float getDistance() {
  delay(500);
  float readings[NUM_SAMPLES];
  for (int i = 0; i < NUM_SAMPLES; i++) {
    readings[i] = trigger_and_record();
    Serial.println(readings[i]);
    delay(50);
  }

  int newSize = NUM_SAMPLES;
  eliminateOutliers(readings, newSize);
  float sum = 0;
  for (int i = 0; i < newSize; i++) {
    sum += readings[i];
  }

  return sum / newSize;
}

float readWallLeft() {
  turnServoLeft90();
  return getDistance();
}

float readWallRight() {
  turnServoRight90();
  return getDistance();
}

float readWallRight85() {
  myservo.write(5);
  return getDistance();
}

void rotation_correction_parallel(float left, float right) {

  float sum = left + right + sonar_separation;

  // Avoid domain error in acos if sum < 18
  if (sum <= wall_distance || sum >= 40) {
    turnServoStraight();
    return;
  }

  // Angle in radians → convert to degrees
  float angle_rad = acos(wall_distance / sum);
  float angle_deg = angle_rad * 180.0 / PI;

  float right85 = readWallRight85();
  delay(300);

  turnServoStraight();
  if (angle_deg > 50) {
    return;
  }

  // Correction direction
  if (right85 < right) {
    // Misaligned to the right → rotate left
    turnLeft(t90l / 90 * angle_deg);
  } else {
    // Misaligned to the left → rotate right
    turnRight(t90r / 90 * angle_deg);
  }
}

void executePath(const char* path) {
  for (int i = 0; path[i] != '\0'; i++) {
    char cmd = path[i];
    switch (cmd) {
      case 'F':
      case 'f':
        moveForward(udt);
        if (i >= 9) {
          }
        break;
      case 'R':
      case 'r':
        turnRight(t90r);
        break;
      case 'L':
      case 'l':
        turnLeft(t90l);
        break;
      default:
        break;
    }
    delay(200);
  }
}

void solveKnownMaze() {
  char shortestRoute[] = "FFLFRFLFLFFFRFRFFFF";
  // char shortestRoute[] = "FFRFFLFFRFRFFFF";
  executePath(shortestRoute);
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
  turnServoStraight();

  Serial.begin(9600);
}

void loop() {
  delay(2000);
  solveKnownMaze();
  delay(999999);
}