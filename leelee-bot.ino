#include "project.h"

#include <Servo.h>

// echo sensor
#define ECHO_ECHO 4  // they grey wire on this pin has issues...
#define ECHO_TRIG 7
#define ECHO_VCC 5  // set this to 5V
// constants
#define ECHO_TIMEOUT 100000L    // how long to wait for response
#define ECHO_TRIG_US 1L         // duration of trigger signal
#define ECHO_CM_PER_US 0.01723  // centimeters per microsecond (as returned by sensor)

// swivel servo
#define SERVO_PIN 3

// motor driver
#define MOTOR_IN1 11
#define MOTOR_IN2 10
#define MOTOR_IN3 9
#define MOTOR_IN4 6

// helpful?
#define MOTOR_LEFT MOTOR_IN1, MOTOR_IN2
#define MOTOR_RIGHT MOTOR_IN3, MOTOR_IN4

Servo swivel;

// Read the echo sensor and return the distance reading in microseconds. Multiply by
// `ECHO_CM_PER_US` to convert from microseconds to centimeters.
unsigned long echo_get_dist() {
  digitalWrite(ECHO_TRIG, HIGH);
  delayMicroseconds(ECHO_TRIG_US);
  digitalWrite(ECHO_TRIG, LOW);

  return pulseIn(ECHO_ECHO, HIGH, ECHO_TIMEOUT);
}

// State of motor according to L298N motor driver module.
typedef enum MOTOR_STATE {
  MOTOR_OFF = 0b00,
  MOTOR_BACKWARD = 0b01,
  MOTOR_FORWARD = 0b10,
  MOTOR_OFF2 = 0b11,  // afaik this does the same thing, but check real dual h-bridge drivers
} MOTOR_STATE;
// this motor driver is a lot less intuitive than the one(s?) I've used before

// Set a motor specified by pins in1 and in2 to a given `MOTOR_STATE`. `MOTOR_LEFT` and `MOTOR_RIGHT` can be
// used in place of pins as convenience: `motor_set_state(MOTOR_LEFT, state)`.
void motor_set_state(uint8_t in1, uint8_t in2, char state) {
  bool in1_on = state & 0b10;
  bool in2_on = state & 0b01;
  digitalWrite(in1, in1_on ? HIGH : LOW);
  digitalWrite(in2, in2_on ? HIGH : LOW);
}

void test_servo() {
  Serial.println("\nServo sweep from 0 to 180");
  delay(500);
  for (int angle = 0; angle <= 180; angle++) {
    swivel.write(angle);
    Serial.print("angle: ");
    Serial.println(angle);
    delay(10);
  }
}

void test_echo() {
  Serial.println("\nEcho test. Does it look good enough?");
  delay(500);
  for (int i = 0; i < 75; i++) {
    Serial.print("dist: ");
    Serial.println(echo_get_dist() * ECHO_CM_PER_US);
    delay(100);
  }
}

void test_swivel_echo() {
  Serial.println("\nServo sweep from 0 to 180, echo along");
  delay(500);

  float dist_acc = 0;

  for (int angle = 0; angle <= 180; angle += 2) {
    swivel.write(angle);
    delay(20);
    float dist = echo_get_dist() * ECHO_CM_PER_US;
    dist_acc += dist;

    Serial.print("angle: ");
    Serial.print(angle);
    Serial.print("; dist: ");
    Serial.println(dist);
  }

  Serial.print("average distance: ");
  Serial.println(dist_acc / 60);
}

void test_motors() {
  Serial.println("\nMotor test: fwd, back, left, right");
  delay(500);

  Serial.println("forward");
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
  delay(200);
  motor_set_state(MOTOR_LEFT, MOTOR_FORWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_FORWARD);
  delay(1000);

  Serial.println("backward");
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
  delay(200);
  motor_set_state(MOTOR_LEFT, MOTOR_BACKWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_BACKWARD);
  delay(1000);

  Serial.println("left");
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
  delay(200);
  motor_set_state(MOTOR_LEFT, MOTOR_BACKWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_FORWARD);
  delay(1000);

  Serial.println("right");
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
  delay(200);
  motor_set_state(MOTOR_LEFT, MOTOR_FORWARD);
  motor_set_state(MOTOR_RIGHT, MOTOR_BACKWARD);
  delay(1000);

  Serial.println("done");
  motor_set_state(MOTOR_LEFT, MOTOR_OFF);
  motor_set_state(MOTOR_RIGHT, MOTOR_OFF);
}

void test() {
  test_servo();
  delay(1000);

  test_echo();
  delay(1000);

  test_swivel_echo();
  delay(1000);

  test_motors();
  delay(1000);
}

void test_desired_print_help() {
  Serial.println(
    "Enter in a character or sequence of characters to test the corresponding function\r\n"
    "\t?: Show help\r\n"
    "\tq: Finish testing\r\n"
    "\ts: Servo swivel\r\n"
    "\te: Echo\r\n"
    "\tl: Swivel + Echo\r\n"
    "\tm: motor\r\n"
    "\tr: reset\r\n"
    "\tn: set servo to degree\r\n"
    "");
}

void test_desired() {
  test_desired_print_help();

  while (true) {
    switch (Serial.read()) {
      case '?':
        test_desired_print_help();
        continue;
      case 'q':
        Serial.println("Quit.");
        return;
      case 'r':
        swivel.write(90);
        continue;

      // no input
      case -1:
        delay(10);
        continue;

      case 's':
        test_servo();
        break;
      case 'e':
        test_echo();
        break;
      case 'l':
        test_swivel_echo();
        break;
      case 'm':
        test_motors();
        break;
      case 'n':
        swivel.write(Serial.parseInt());
        break;

      default:
        continue;
    }

    Serial.println("Test complete.");
  }
}

void setup() {
  pinMode(ECHO_ECHO, INPUT);
  pinMode(ECHO_TRIG, OUTPUT);
  pinMode(ECHO_VCC, OUTPUT);

  digitalWrite(ECHO_VCC, HIGH);  // sigh

  pinMode(SERVO_PIN, OUTPUT);
  swivel.attach(SERVO_PIN);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  Serial.begin(BAUD);
  swivel.write(90);

  //test_desired();
}

void loop() {
  test_motors();
}
