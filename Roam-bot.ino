#include <Servo.h>

/* PIN DEFINITIONS */
// Motors (L298N)
#define MOTOR_IN1 11
#define MOTOR_IN2 10
#define MOTOR_IN3 9
#define MOTOR_IN4 6
#define MOTOR_LEFT MOTOR_IN1, MOTOR_IN2
#define MOTOR_RIGHT MOTOR_IN3, MOTOR_IN4

// Ultrasonic sensor
#define ECHO_ECHO 4
#define ECHO_TRIG 7
#define ECHO_VCC 5

// Servo
#define SERVO_PIN 3

// Buzzer
#define BUZZER_PIN 8

/* CONSTANTS */
#define DISTANCE_THRESHOLD 12   // cm
#define SAMPLE_COUNT 7
#define ECHO_TIMEOUT 30000      // us
#define ECHO_TRIG_US 10         // us
#define ECHO_CM_PER_US 0.01723

/* GLOBALS */
Servo swivel;
float distanceSamples[SAMPLE_COUNT];
int sampleIndex = 0;

/* MOTOR STATE ENUM */
typedef enum MOTOR_STATE {
  MOTOR_OFF = 0b00,
  MOTOR_BACKWARD = 0b01,
  MOTOR_FORWARD = 0b10,
  MOTOR_OFF2 = 0b11
} MOTOR_STATE;

/* SETUP */
void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  pinMode(ECHO_ECHO, INPUT);
  pinMode(ECHO_TRIG, OUTPUT);
  pinMode(ECHO_VCC, OUTPUT);
  digitalWrite(ECHO_VCC, HIGH);

  swivel.attach(SERVO_PIN);
  swivel.write(98);

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(9600);

  clearSamples();
  stopMotors();

  Serial.println("Roam-Bot v1 started");
}

/* MAIN LOOP */
void loop() {
  float distance = echo_get_dist() * ECHO_CM_PER_US;
  storeSample(distance);

  float maxDist = getMaxSample();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Max(");
  Serial.print(SAMPLE_COUNT);
  Serial.print("): ");
  Serial.println(maxDist);

  if (maxDist > DISTANCE_THRESHOLD) {
    moveForward();
  } else {
    beep(2000, 150);
    handleObstacle();
  }

  delay(50);
}

/* ULTRASONIC */
unsigned long echo_get_dist() {
  digitalWrite(ECHO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ECHO_TRIG, HIGH);
  delayMicroseconds(ECHO_TRIG_US);
  digitalWrite(ECHO_TRIG, LOW);

  return pulseIn(ECHO_ECHO, HIGH, ECHO_TIMEOUT);
}

/* DISTANCE BUFFER */
void storeSample(float value) {
  distanceSamples[sampleIndex] = value;
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
}

float getMaxSample() {
  float maxVal = distanceSamples[0];
  for (int i = 1; i < SAMPLE_COUNT; i++) {
    if (distanceSamples[i] > maxVal) maxVal = distanceSamples[i];
  }
  return maxVal;
}

void clearSamples() {
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    distanceSamples[i] = 300;
  }
}

/* BUZZER */
void beep(int frequency, int duration) {
  tone(BUZZER_PIN, frequency);
  delay(duration);
  noTone(BUZZER_PIN);
}

/* MOTOR CONTROL */
void motor_set_state(uint8_t in1, uint8_t in2, char state) {
  digitalWrite(in1, (state & 0b10) ? HIGH : LOW);
  digitalWrite(in2, (state & 0b01) ? HIGH : LOW);
}

void moveForward() {
  motor_set_state(MOTOR_LEFT, MOTOR_FORWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_FORWARD);
}

void moveBackward() {
  motor_set_state(MOTOR_LEFT, MOTOR_BACKWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void pivotLeft() {
  motor_set_state(MOTOR_LEFT, MOTOR_BACKWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_FORWARD);
}

void pivotRight() {
  motor_set_state(MOTOR_LEFT, MOTOR_FORWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void stopMotors() {
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
}

/* OBSTACLE HANDLING */
void handleObstacle() {
  Serial.println("Obstacle detected!");

  stopMotors();
  delay(100);

  // Back up
  Serial.println("Backing up...");
  moveBackward();
  delay(400);
  stopMotors();
  delay(100);

  // Scan
  int turnDir = scanForBestDirection();

  if (turnDir < 0) {
    Serial.println("Pivoting LEFT");
    pivotLeft();
  } else {
    Serial.println("Pivoting RIGHT");
    pivotRight();
  }

  // turning
  unsigned long startTime = millis();
  bool minimumTurnMet = false;

  while (millis() - startTime < 1500) {
    float d = echo_get_dist() * ECHO_CM_PER_US;

    if (millis() - startTime > 600) {
      minimumTurnMet = true;
    }

    Serial.print("Turning distance: ");
    Serial.println(d);

    if (minimumTurnMet && d > DISTANCE_THRESHOLD + 5) {
      break;
    }
  }

  stopMotors();
  swivel.write(98);
  clearSamples();
}

/* SERVO SCAN */
int scanForBestDirection() {
  swivel.write(180);
  delay(300);
  float leftDist = echo_get_dist() * ECHO_CM_PER_US;
  Serial.print("Left scan: ");
  Serial.println(leftDist);

  swivel.write(0);
  delay(300);
  float rightDist = echo_get_dist() * ECHO_CM_PER_US;
  Serial.print("Right scan: ");
  Serial.println(rightDist);

  return (leftDist > rightDist) ? -1 : 1;
}
