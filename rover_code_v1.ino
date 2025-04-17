const int LMotorEn = 5;
const int LMotor1 = 8;
const int LMotor2 = 9;
const int RMotorEn = 6;
const int RMotor1 = 10;
const int RMotor2 = 11;

double lms = 255;  // left motor's speed (0-255)
double rms = 255;  // right motor's speed (0-255)
double udt = 2470; // time taken to travel unit distance in the maze = 18.5cm (for now)
double t90 = 945;  // turning time for 90 degrees

void turnRight(int duration) {
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);
  digitalWrite(RMotor1, HIGH);
  digitalWrite(RMotor2, LOW);
  digitalWrite(LMotorEn, lms);
  digitalWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void turnLeft(int duration) {
  digitalWrite(LMotor1, LOW);
  digitalWrite(LMotor2, HIGH);
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
  digitalWrite(LMotorEn, lms);
  digitalWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void moveForward(int duration) {
  digitalWrite(LMotor1, HIGH);
  digitalWrite(LMotor2, LOW);
  digitalWrite(RMotor1, LOW);
  digitalWrite(RMotor2, HIGH);
  digitalWrite(LMotorEn, lms);
  digitalWrite(RMotorEn, rms);
  delay(duration);
  stopMotors();
}

void stopMotors() {
  digitalWrite(LMotorEn, LOW);
  digitalWrite(RMotorEn, LOW);
}

void executePath(const char* path) {
  for (int i = 0; path[i] != '\0'; i++) {
    char cmd = path[i];
    switch (cmd) {
      case 'F':
      case 'f':
        moveForward(udt);
        break;
      case 'R':
      case 'r':
        turnRight(t90);
        break;
      case 'L':
      case 'l':
        turnLeft(t90);
        break;
      default:
        // Unknown command, ignore or print error
        break;
    }
  }
}

void solveKnownMaze() {
  char shortestRoute[] = "FFLFRFLFLFFFRFRFFF";
  executePath(shortestRoute);
}

void setup() {
  pinMode(LMotorEn, OUTPUT);
  pinMode(LMotor1,  OUTPUT);
  pinMode(LMotor2,  OUTPUT);
  pinMode(RMotorEn, OUTPUT);
  pinMode(RMotor1,  OUTPUT);
  pinMode(RMotor2,  OUTPUT);
}

void loop() {
  delay(200);
  solveKnownMaze();
  delay(99999);
}
