#include "Servo.h"
#include <math.h>

const float sonar_separation = 3.2;
const float wall_distance = 18.0; // Maze wall spacing in cm
const int NUM_SAMPLES = 1;
const float WALL_THRESHOLD = 15;

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

double lms = 250;  // left motor's speed (0-255)
double rms = 255;  // right motor's speed (0-255)
double udt = 2725; // time taken to travel unit distance in the maze = 18.5 cm (for now)
double t90 = 1050;  // turning time for 90 degrees

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

void turnServoLeft90() {
  myservo.write(180);
  delay(200);
} 

void turnServoLeft45() {
  myservo.write(132);
  delay(200);
} 

void turnServoStraight() {
  myservo.write(87);
  delay(200);
}

void turnServoRight45() {
  myservo.write(42);
  delay(200);
} 

void turnServoRight90() {
  myservo.write(0);
  delay(200);
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

float trigger_and_record() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH, 20000); // timeout after 20ms
  return duration * 0.034 / 2 - 0.4; // Convert to cm
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
  // eliminateOutliers(readings, newSize);

  float sum = 0;
  for (int i = 0; i < newSize; i++) {
    sum += readings[i];
  }

  return sum / newSize;
}

void eliminateOutliers(float *arr, int &size) {
  // Sort array (simple bubble sort for Arduino-sized arrays)
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }

  // Remove min and max (first and last)
  for (int i = 1; i < size - 1; i++) {
    arr[i - 1] = arr[i];
  }

  size = size - 2;
}

bool checkWallInfront() {
  // Check straight wall normally
  turnServoStraight();
  delay(200);
  moveBackward(500);
  float d = getDistance();
  moveForward(500);
  if ((d < WALL_THRESHOLD) && (d > 1)) {
    return true;
  }

  // Check right diagonal (45°)
  turnServoRight45();
  delay(200);
  d = getDistance();
  if ((d < WALL_THRESHOLD) && (d > 1)) {
    // Probe forward 1 cm to confirm it's not a pole
    moveForward(350);  // Move forward 1 cm
    delay(200);
    float d2 = getDistance();
    moveBackward(350); // Move back to original position

    if ((d2 < WALL_THRESHOLD) && (d2 > 1)) {
      turnServoStraight();
      return true;  // Confirmed diagonal wall
    }
  }

  // Check left diagonal (45°)
  turnServoLeft45(); // This brings it back to center
  delay(200);
  d = getDistance();
  if ((d < WALL_THRESHOLD) && (d > 1)) {
    // Probe forward 1 cm to confirm
    moveForward(350);
    delay(200);
    float d2 = getDistance();
    moveBackward(350);

    if ((d2 < WALL_THRESHOLD) && (d2 > 1)) {
      turnServoStraight();
      return true;  // Confirmed diagonal wall
    }
  }

  // Reset servo to straight after all checks
  turnServoStraight();
  return false;
}

int x = 0;
int y = 0;

enum Direction {NORTH, EAST, SOUTH, WEST};
Direction facing = NORTH;

void updatePosition() {
  switch (facing) {
    case NORTH: y++; break;
    case EAST:  x--; break;
    case SOUTH: y--; break;
    case WEST:  x++; break;
  }
}

void updateFacingLeft() {
  facing = static_cast<Direction>((facing + 3) % 4); // turnLeft = -1 mod 4
}

void updateFacingRight() {
  facing = static_cast<Direction>((facing + 1) % 4); // turnRight = +1 mod 4
}

void solveMaze() {
  facing = NORTH;
  while (!(x == 3 && y == 3)) {
    if (!checkWallInfront()) {
      float left_d = readWallLeft();
      float right_d = readWallRight();
      if (left_d < WALL_THRESHOLD && right_d < WALL_THRESHOLD) {
        rotation_correction_parallel(left_d, right_d);
      }
      moveForward(udt);
      updatePosition();
      continue;
    }

    // Try left
    turnLeft(t90);
    updateFacingLeft();
    delay(100);
    if (!checkWallInfront()) {
      moveForward(udt);
      updatePosition();
      continue;
    }

    // Back to original, then try right
    turnRight(t90);
    delay(200);
    turnRight(t90);
    updateFacingRight(); updateFacingRight();
    delay(100);
    if (!checkWallInfront()) {
      moveForward(udt);
      updatePosition();
      continue;
    }

    // Dead end: reverse and turn right
    moveBackward(udt);
    turnRight(t90);
    updateFacingRight();
    moveForward(udt);
    updatePosition();
  }

  stopMotors();
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

// Perform angle correction using trigonometry
void rotation_correction_parallel(float left, float right) {
  Serial.println("here's left measurement");
  Serial.println(left);
  Serial.println("here's right measurement");
  Serial.println(right);
  float sum = left + right + sonar_separation;
  Serial.println("here's sum cal");
  Serial.println(sum);
  // Avoid domain error in acos if sum < 18
  if (sum <= wall_distance) {
    turnServoStraight();
    return;
  }

  // Angle in radians → convert to degrees
  float angle_rad = acos(wall_distance / sum);
  float angle_deg = angle_rad * 180.0 / PI;
  Serial.println("here's angle cal");
  Serial.println(angle_deg);
  float right80 = readWallRight85();
  delay(300);
  Serial.println("here's right80");
  Serial.println(right80);
  turnServoStraight();
  if (angle_deg > 40) {
    return;
  }

  // Correction direction
  if (right80 < right) {
    // Misaligned to the right → rotate left
    Serial.println("turned left by ");
    turnLeft(t90 / 90 * angle_deg);
  } else {
    // Misaligned to the left → rotate right
    Serial.println("turned right by ");
    turnRight(t90 / 90 * angle_deg);
  }

  turnServoStraight();  // Reset servo
}

void loop() {
  delay(2000);
  solveMaze();
  delay(20000);
}
