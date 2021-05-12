#include <Arduino.h>
#include "util.h"
#include "Subsystems.h"
#include "pins.h"
#include "DSState.h"
#include "DSProtocol.h"
#include "Adafruit_PWMServoDriver.h"

/**
 * ServoBlock
 */
#define SERVOMIN  120
#define SERVOMAX  500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
void ServoBlock::setup () {
  pwm = Adafruit_PWMServoDriver();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}

void ServoBlock::set_angle (int servo, uint32_t angle) {
  uint32_t pl = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPin(servo, pl);
}

void ServoBlock::set_pulse (int servo, int pulse) {
  pwm.setPin(servo, pulse);
}

/**
 * Elevator
 */
void Elevator::setup (ServoBlock* servos) {
  sb = servos;
}

void Elevator::set_height (float height) {
}

float Elevator::get_height () {
  return h;
}

float Elevator::get_error () {
  return error;
}

float Elevator::get_inferred_height () {
  return h_hat;
}

void Elevator::tick (float delta) {
  // Calculate error
  error = h - h_hat;
  // Update w
  if (error > DAMPING_FACTOR) {
    set_direction(ElevatorDirection::Up);
  }
  if (error < -DAMPING_FACTOR) {
    set_direction(ElevatorDirection::Down);
  }
  // Update inferred height
  h_hat += (float)DRIVER_RADIUS * a_vel * delta;
}

void Elevator::set_direction (ElevatorDirection direction) {
  dir = direction;

  switch (dir) {
    case ElevatorDirection::Up:
      a_vel = DRIVER_VELOCITY;
      //sb->set_angle(P_ELEVATOR, ELEVATOR_MAX_ANGLE);
      sb->set_pulse(P_ELEVATOR_1, 4096);
      sb->set_pulse(P_ELEVATOR_2, 0);
      break;
    case ElevatorDirection::Down:
      a_vel = -DRIVER_VELOCITY;
      sb->set_pulse(P_ELEVATOR_1, 0);
      sb->set_pulse(P_ELEVATOR_2, 4096);
      //sb->set_angle(P_ELEVATOR, ELEVATOR_MIN_ANGLE);
      break;
    case ElevatorDirection::Hold:
      a_vel = 0;
      sb->set_pulse(P_ELEVATOR_1, 0);
      sb->set_pulse(P_ELEVATOR_2, 0);
      //sb->set_angle(P_ELEVATOR, 90);
      break;
  }
}

/**
 * Grabber
 */
void Grabber::setup (ServoBlock* servos) {
  sb = servos;
}

void Grabber::set_grip (float grip) {
  g = clamp(grip, 0.f, 1.f);
  float angle = map(grip, 0.f, 1.f, (float)GRABBER_MIN_ANGLE, (float)GRABBER_MAX_ANGLE);
  int iangle = static_cast<int>(angle);
  sb->set_angle(P_GRABBER, (uint32_t)iangle);
}

float Grabber::get_grip () {
  return g;
}

bool Grabber::is_gripped () {
  return g > 0.5f; // TODO: What?
}

void Grabber::open () {
  set_grip(GRABBER_OPEN);
}

void Grabber::close () {
  set_grip(GRABBER_GRAB);
}

void Grabber::toggle () {
  set_grip(is_gripped() ? 0.f : 1.f);
}
/**
 * DSInterface
 */
void DSInterface::setup () { }

void DSInterface::poll () {
  new_packet = protocol.process();
  if (new_packet) {
    enabled = protocol.getStatus().enabled && !protocol.getStatus().estopped;
  }
}

float DSInterface::get_axis (GamepadAxis axis) {
  return protocol.getStatus().gamepad.getAxisFloat(axis);
}

bool DSInterface::get_button (GamepadButton button) {
  return protocol.getStatus().gamepad.getButton(button);
}

/*
 * Drivetrain
 */

void Drivetrain::setup () {
  // Init speed
  pinMode(P_LEFT_SPEED, OUTPUT);
  pinMode(P_RIGHT_SPEED, OUTPUT);
  pinMode(P_LEFT_1, OUTPUT);
  pinMode(P_LEFT_2, OUTPUT);
  pinMode(P_RIGHT_1, OUTPUT);
  pinMode(P_RIGHT_2, OUTPUT);
  setPower(0.0, 0.0);
}

// "Power" is -1.0 to +1.0
// This is equivalent to the WPILib TankDrive mode
void Drivetrain::setPower(double left, double right) {
  setPower(LEFT, left);
  setPower(RIGHT, right);
}

void Drivetrain::setPower(int side, double power) {
  if (power > 0) {
    setDirection(side, FORWARD);
  }
  if (power < 0) {
    setDirection(side, BACKWARD);
  }
  if (power == 0) {
    // Force power to 1 to power brake
    // This is probably undesirable, but we'll see
    setDirection(side, BRAKE);
    setSpeed(side, MAX_SPEED);
    return;
  }
  // Denormalize speed
  int denormalized = (int)(abs(power) * MAX_SPEED);
  setSpeed(side, denormalized);
}

void Drivetrain::setDirection(int side, int direction) {
  if (side == LEFT) {
    if (direction == FORWARD) {
      digitalWrite(P_LEFT_1, HIGH);
      digitalWrite(P_LEFT_2, LOW);
    }
    if (direction == BACKWARD) {
      digitalWrite(P_LEFT_1, LOW);
      digitalWrite(P_LEFT_2, HIGH);
    }
    if (direction == BRAKE) {
      digitalWrite(P_LEFT_1, HIGH);
      digitalWrite(P_LEFT_2, HIGH);
    }
  }
  if (side == RIGHT) {
    if (direction == FORWARD) {
      digitalWrite(P_RIGHT_1, LOW);
      digitalWrite(P_RIGHT_2, HIGH);
    }
    if (direction == BACKWARD) {
      digitalWrite(P_RIGHT_1, HIGH);
      digitalWrite(P_RIGHT_2, LOW);
    }
    if (direction == BRAKE) {
      digitalWrite(P_RIGHT_1, HIGH);
      digitalWrite(P_RIGHT_2, HIGH);
    }
  }
}

void Drivetrain::setSpeed(int side, int speed) {
  if (side == LEFT) {
    analogWrite(P_LEFT_SPEED, speed);
  }
  if (side == RIGHT) {
    analogWrite(P_RIGHT_SPEED, speed);
  }
}

/*
 * Linetracker
 */

void Linetracker::setup() {
  pinMode(P_LEFT, INPUT);
  pinMode(P_CENTER, INPUT);
  pinMode(P_RIGHT, INPUT);
}

bool Linetracker::left() {
  return !digitalRead(P_LEFT);
}

bool Linetracker::center() {
  return !digitalRead(P_CENTER);
}

bool Linetracker::right() {
  return !digitalRead(P_RIGHT);
}

bool Linetracker::all() {
  return left() && right() && center();
}

bool Linetracker::any() {
  return left() || right() || center();
}
