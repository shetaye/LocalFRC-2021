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
void ServoBlock::set_angle (int servo, uint32_t angle) {
  uint32_t pl = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPin(servo, pl);
}

void ServoBlock::set_pulse (int servo, int pulse) {
  int pl = clamp(pulse, SERVOMIN, SERVOMAX);
  pwm.setPin(servo, pl);
}

/**
 * MPU
 */
#define SHRT_MIN -32768
#define SHRT_MAX 32768
void apply_ema(float* last_sample, float sample, float alpha) { *last_sample = sample * alpha + *last_sample * (1.f - alpha); }
float sample_to_g(int16_t sample) { return map((float)sample, -32768.f, 32768.f, -2.f, 2.f); }
void Mpu::poll (float delta) {
  int16_t t_pax, t_pay, t_paz, t_gax, t_gay, t_gaz = 0;
  mpu.getMotion6(&t_pax, &t_pay, &t_paz, &t_gax, &t_gay, &t_gaz);
  // Apply deadzone
  t_pax = deadzone(t_pax, MPU_DEADZONE);
  t_pay = deadzone(t_pay, MPU_DEADZONE);
  t_paz = deadzone(t_paz, MPU_DEADZONE);
  t_gax = deadzone(t_gax, MPU_DEADZONE);
  t_gay = deadzone(t_gay, MPU_DEADZONE);
  t_gaz = deadzone(t_gaz, MPU_DEADZONE);
  // Convert to Gs
  float f_pax = sample_to_g(t_pax);
  float f_pay = sample_to_g(t_pay);
  float f_paz = sample_to_g(t_paz);
  float f_gax = sample_to_g(t_gax);
  float f_gay = sample_to_g(t_gay);
  float f_gaz = sample_to_g(t_gaz);
  // Update moving average
#ifdef MPU_EMA
  float alpha = (float)MPU_EMA_SMOOTHING / (1.f + MPU_EMA_SAMPLES);
  apply_ema(&p_ax, f_pax, alpha);
  apply_ema(&p_ay, f_pay, alpha);
  apply_ema(&p_az, f_paz, alpha);
  apply_ema(&g_ax, f_gax, alpha);
  apply_ema(&g_ay, f_gay, alpha);
  apply_ema(&g_az, f_gaz, alpha);
#else
  p_ax = f_pax;
  p_ay = f_pay;
  p_az = f_paz;
  g_ax = f_gax;
  g_ay = f_gay;
  g_az = f_gaz;
#endif
  // Integrate over acceleration
#ifdef MPU_EMA
  apply_ema(&p_vx, p_ax * delta, alpha);
  apply_ema(&p_vy, p_ay * delta, alpha);
  apply_ema(&p_vz, p_az * delta, alpha);
  apply_ema(&g_vx, g_ax * delta, alpha);
  apply_ema(&g_vy, g_ay * delta, alpha);
  apply_ema(&g_vz, g_az * delta, alpha);
#else
  p_vx += p_ax * delta;
  p_vy += p_ay * delta;
  p_vz += p_az * delta;
  g_vx += g_ax * delta;
  g_vy += g_ay * delta;
  g_vz += g_az * delta;
#endif
  // Integrate over velocity
  p_x += p_vx * delta;
  p_y += p_vy * delta;
  p_z += p_vz * delta;
  g_x += g_vx * delta;
  g_y += g_vy * delta;
  g_z += g_vz * delta;
}
void Mpu::zero () {
  p_vx = p_vy = p_vz = 0;
  g_vx = g_vy = g_vz = 0;

  p_x = p_y = p_z = 0;
  g_x = g_y = g_z = 0;
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

/*
 * Ultrasonic
 */

