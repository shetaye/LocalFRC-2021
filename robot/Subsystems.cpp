#include <Arduino.h>
#include "I2Cdev.h"
#include "util.h"
#include "Subsystems.h"
#include "pins.h"
#include "DSState.h"
#include "DSProtocol.h"
#include "Adafruit_PWMServoDriver.h"

/**
 * ServoBlock
 */
void ServoBlock::set_angle (int servo, uint32_t angle) {
  uint32_t pl = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPin(servo, pl);
}

void ServoBlock::set_pulse (int servo, int pulse) {
  int pl = clamp(pulse, SERVOMIN, SERVOMAX);
  pwm.setPin(servo, pl);
}

void ServoBlock::begin () {
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}

/**
 * Elevator
 */
void Elevator::zero () {
  h_hat = 0;
}

float Elevator::get_inferred_height () {
  return h_hat;
}

void Elevator::tick (float delta) {
  // Update inferred height
  h_hat += DRIVER_RADIUS * a_vel * delta;
}

void Elevator::set_direction (ElevatorDirection direction) {
  dir = direction;

  switch (dir) {
    case ElevatorDirection::Up:
      a_vel = DRIVER_VELOCITY;
      //sb->set_angle(P_ELEVATOR, ELEVATOR_MAX_ANGLE);
      sb->pwm.setPin(P_ELEVATOR_1, 4096);
      sb->pwm.setPin(P_ELEVATOR_2, 0);
      break;
    case ElevatorDirection::Down:
      a_vel = -DRIVER_VELOCITY;
      sb->pwm.setPin(P_ELEVATOR_1, 0);
      sb->pwm.setPin(P_ELEVATOR_2, 4096);
      //sb->set_angle(P_ELEVATOR, ELEVATOR_MIN_ANGLE);
      break;
    case ElevatorDirection::Hold:
      a_vel = 0;
      sb->pwm.setPin(P_ELEVATOR_1, 0);
      sb->pwm.setPin(P_ELEVATOR_2, 0);
      //sb->set_angle(P_ELEVATOR, 90);
      break;
  }
}

/**
 * Ultrasonic
 */
void Ultrasonic::ping () {
  digitalWrite(P_UTRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(P_UTRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(P_UTRIG, LOW);

  float duration = pulseIn(P_UECHO, HIGH);
  distance = duration / 29 / 2;
}

void Ultrasonic::begin () {
  pinMode(P_UTRIG, OUTPUT);
  pinMode(P_UECHO, INPUT);
}

/**
 * Wrist
 */
void Wrist::set_angle (float angle) {
  a = clamp(angle, -1.f, 1.f);
  float f_pl = map(a, -1.f, 1.f, (float)WRIST_MIN_PULSE, (float)WRIST_MAX_PULSE);
  int pl = static_cast<int>(f_pl);
  sb->set_pulse(P_WRIST_1, pl);
  sb->set_pulse(P_WRIST_2, pl);
}

float Wrist::get_angle () {
  return a;
}

void Wrist::up() {
  set_angle(WRIST_UP);
}

void Wrist::hold() {
  set_angle(WRIST_HOLD);
}

void Wrist::down() {
  set_angle(WRIST_DOWN);
}

/**
 * Grabber
 */
void Grabber::set_grip (float grip) {
  g = clamp(grip, 0.f, 1.f);
  float f_pl = map(g, 0.f, 1.f, (float)GRABBER_MIN_PULSE, (float)GRABBER_MAX_PULSE);
  int pl = static_cast<int>(f_pl);
  sb->set_pulse(P_GRABBER, pl);
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

bool DSInterface::get_button (int button) {
  return protocol.getStatus().gamepad.buttonState & button;
}

/*
 * Drivetrain
 */
void Drivetrain::begin() {
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

void Linetracker::begin() {
  pinMode(P_LEFT, INPUT);
  pinMode(P_CENTER, INPUT);
  pinMode(P_RIGHT, INPUT);
}

/**
 * MPU
 */
void Mpu::poll() {
  if (!dmp_ready) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifo_buffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
}
void Mpu::begin() {
  // Zero FIFO
  for (uint8_t i = 0; i < 64; i++) { fifo_buffer[i] = 0; }

  mpu.initialize();
  uint8_t device_status = mpu.dmpInitialize();
  if (device_status == 0)  {
    mpu.setXGyroOffset(MPU_X_GYRO_OFFSET);
    mpu.setYGyroOffset(MPU_Y_GYRO_OFFSET);
    mpu.setZGyroOffset(MPU_Z_GYRO_OFFSET);
    mpu.setXAccelOffset(MPU_X_ACCL_OFFSET);
    mpu.setYAccelOffset(MPU_Y_ACCL_OFFSET);
    mpu.setZAccelOffset(MPU_Z_ACCL_OFFSET);

    mpu.setDMPEnabled(true);

    dmp_ready = true;
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(device_status);
    Serial.println(")");
  }
}

