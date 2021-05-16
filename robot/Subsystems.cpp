#include <Arduino.h>
#include "I2Cdev.h"
#include "util.h"
#include "Subsystems.h"
#include "pins.h"
#include "DSState.h"
#include "DSProtocol.h"
#include "Adafruit_PWMServoDriver.h"
#include "MPU_WriteMacros.h"
#include "MPU_ReadMacros.h"
#include "DMP_Image.h"

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

/**
 * MPU
 */
// I2C
int8_t Mpu::MPUi2cRead (uint8_t addr, uint8_t len, uint8_t n_bits, uint8_t* data) {
  return MPUi2cRead(dev_addr, addr, len, n_bits, data);
}
int8_t Mpu::MPUi2cRead (uint8_t alt_addr, uint8_t addr, uint8_t len, uint8_t n_bits, uint8_t* data) {
  if (len == 1) return I2Cdev::readBit(alt_addr, addr, n_bits, data);
  return I2Cdev::readBits(alt_addr, addr, n_bits, len, data);
}
int8_t Mpu::MPUi2cReadByte (uint8_t addr, uint8_t* data) {
  return I2Cdev::readBytes(dev_addr, addr, 1, data);
}
int8_t Mpu::MPUi2cReadByte (uint8_t alt_addr, uint8_t addr, uint8_t* data) {
  return I2Cdev::readBytes(alt_addr, addr, 1, data);
}
int8_t Mpu::MPUi2cReadBytes (uint8_t addr, uint8_t len, uint8_t* data) {
  return I2Cdev::readBytes(dev_addr, addr, len, data);
}
int8_t Mpu::MPUi2cReadBytes (uint8_t alt_addr, uint8_t addr, uint8_t len, uint8_t* data) {
  return I2Cdev::readBytes(alt_addr, addr, len, data);
}
int8_t Mpu::MPUi2cReadInt (uint8_t addr, uint16_t* data) {
  return I2Cdev::readWords(dev_addr, addr, 1, data);
}
int8_t Mpu::MPUi2cReadInt (uint8_t alt_addr, uint8_t addr, uint16_t* data) {
  return I2Cdev::readWords(alt_addr, addr, 1, data);
}
int8_t Mpu::MPUi2cReadInts (uint8_t addr, uint16_t n_words, uint16_t* data) {
  return I2Cdev::readWords(dev_addr, addr, n_words, data);
}
int8_t Mpu::MPUi2cReadInts (uint8_t alt_addr, uint8_t addr, uint16_t n_words, uint16_t* data) {
  return I2Cdev::readWords(alt_addr, addr, n_words, data);
}
int8_t Mpu::MPUi2cWrite (uint8_t addr, uint8_t len, uint8_t n_bits, uint8_t data) {
  return MPUi2cWrite(dev_addr, addr, len, n_bits, data);
}
int8_t Mpu::MPUi2cWrite (uint8_t alt_addr, uint8_t addr, uint8_t len, uint8_t n_bits, uint8_t data) {
  if (len == 1) return I2Cdev::writeBit(alt_addr, addr, n_bits, &data);
  else if (n_bits != 255) return I2Cdev::writeBits(alt_addr, addr, n_bits, len, &data);
  return 0;
}
int8_t Mpu::MPUi2cWriteByte (uint8_t addr, uint8_t data) {
  return I2Cdev::writeBytes(dev_addr, addr, 1, &data);
}
int8_t Mpu::MPUi2cWriteByte (uint8_t alt_addr, uint8_t addr, uint8_t data) {
  return I2Cdev::writeBytes(alt_addr, addr, 1, &data);
}
int8_t Mpu::MPUi2cWriteBytes (uint8_t addr, uint8_t len, uint8_t* data) {
  return I2Cdev::writeBytes(dev_addr, addr, len, data);
}
int8_t Mpu::MPUi2cWriteBytes (uint8_t alt_addr, uint8_t addr, uint8_t len, uint8_t* data) {
  return I2Cdev::writeBytes(alt_addr, addr, len, data);
}
int8_t Mpu::MPUi2cWriteInt (uint8_t addr, uint16_t data) {
  return I2Cdev::writeWords(dev_addr, addr, 1, &data);
}
int8_t Mpu::MPUi2cWriteInt (uint8_t alt_addr, uint8_t addr, uint16_t data) {
  return I2Cdev::writeWords(alt_addr, addr, 1, &data);
}
int8_t Mpu::MPUi2cWriteInts (uint8_t addr, uint16_t n_words, uint16_t* data) {
  return I2Cdev::writeWords(dev_addr, addr, n_words, data);
}
int8_t Mpu::MPUi2cWriteInts (uint8_t alt_addr, uint8_t addr, uint16_t n_words, uint16_t* data) {
  return I2Cdev::writeWords(alt_addr, addr, n_words, data);
}

// Connection
bool Mpu::test_connection_status () {
  Wire.beginTransmission(dev_addr);
  if (Wire.endTransmission() != 0) {
    Serial.print("Nothing at 0x");
    Serial.println(dev_addr);
    return false;
  }
  Serial.print("Found MPU at 0x");
  Serial.println(dev_addr);
  return true;
}

// Interrupts
uint8_t Mpu::check_int () {
  bool trig;
  noInterrupts();
  trig = *mpu_int;
  *mpu_int = false;
  interrupts();
  if (trig) {
    uint8_t stat;
    // Get DMP INT status
    INT_STATUS_READ(&stat);
    return stat;
  }
  return 0;
}

// Memory
void Mpu::read_memory(uint16_t addr, uint16_t length, uint8_t* data) {
  BANK_SEL_WRITE(addr);
  DMP_MEM_READ(length, data);
}
void Mpu::write_memory(uint16_t addr, uint16_t length, uint8_t* data) {
  BANK_SEL_WRITE(addr);
  DMP_MEM_WRITE(length, data);
}

// DMP
bool Mpu::prog_dmp (const uint8_t* prog, uint16_t length) {
  uint16_t bytes_written = 0;
  uint8_t* p_prog;
  uint16_t bank = 0;
  uint8_t prog_chunk[DMP_CHUNK_SIZE];

  if (dmp_loaded) return false;

  for (uint16_t i = 0; i < length; i += bytes_written) {
    bytes_written = min(DMP_CHUNK_SIZE, length - i);
    p_prog = (uint8_t*)&prog[i];
    // Copy a chunk of the firmware
    for (int16_t x = 0; x < bytes_written; x++) {
      prog_chunk[x] = pgm_read_byte_near(p_prog + x);
    }
    // Load that chunk
    write_memory(i, bytes_written, prog_chunk);
  }
  dmp_loaded = true;
}
bool Mpu::load_offsets () {
  XA_OFFSET_H_WRITE_XA_OFFS(xa_offset);
  YA_OFFSET_H_WRITE_YA_OFFS(ya_offset);
  ZA_OFFSET_H_WRITE_ZA_OFFS(za_offset);
  XG_OFFSET_H_WRITE_X_OFFS_USR(xg_offset);
  YG_OFFSET_H_WRITE_Y_OFFS_USR(yg_offset);
  ZG_OFFSET_H_WRITE_Z_OFFS_USR(zg_offset);
}
void Mpu::init_dmp () {
  bool connected = test_connection_status();
  if (!connected) return;

  PWR_MGMT_1_WRITE_DEVICE_RESET();
  MPUi2cWriteByte(0x6B, 0x00);
  MPUi2cWriteByte(0x6C, 0x00);
  MPUi2cWriteByte(0x1A, 0x03);
  MPUi2cWriteByte(0x1B, 0x18);
  MPUi2cWriteByte(0x1C, 0x00);
  MPUi2cWriteByte(0x23, 0x00);
  MPUi2cWriteByte(0x38, 0x00);
  MPUi2cWriteByte(0x6A, 0x04);
  MPUi2cWriteByte(0x19, 0x04);
  prog_dmp(dmp_memory, DMP_CODE_SIZE);
  MPUi2cWriteInt(0x70, 0x0400);
  load_offsets();
  MPUi2cWriteByte(0x6A, 0xC0);
  MPUi2cWriteByte(0x38, 0x02);
  dmp_ready = true;
}

// FIFO
int16_t Mpu::get_fifo_count () {
  int16_t c;
  FIFO_COUNTH_READ_FIFO_CNT(&c);
  return c;
}
int8_t Mpu::get_fifo_packet(uint8_t* data) {
  int16_t fifo_count;
  uint32_t break_timer = micros();
  do {
    if ((fifo_count=get_fifo_count()) > DMP_PACKET_LENGTH) {
      if (fifo_count > 200) {
        // Too many packets, so we reset
        USER_CTRL_WRITE_FIFO_RST ();
        fifo_count = 0;
        // Wait for next packet
        while (!(fifo_count = get_fifo_count()) && ((micros() - break_timer) <= 11000));
      } else {
        uint8_t trash[BUFFER_LENGTH];
        // DMP could be writing as we speak so we check every time
        while((fifo_count = get_fifo_count()) > DMP_PACKET_LENGTH) {
          fifo_count -= DMP_PACKET_LENGTH;
          uint16_t remove_bytes;
          while (fifo_count) {
            remove_bytes = min(fifo_count, BUFFER_LENGTH);
            FIFO_READ((uint8_t)remove_bytes, trash);
            fifo_count -= remove_bytes;
          }
        }
      }
    }
    if (!fifo_count) return 0;
    if ((micros() - break_timer) > 11000) return 0;
  } while (fifo_count != DMP_PACKET_LENGTH);
  FIFO_READ(DMP_PACKET_LENGTH, data);
  return 1;
}
void Mpu::check_fifo () {
  uint8_t interrupt_status = check_int();
  if (!interrupt_status) return;
  Serial.println(interrupt_status);

  if(get_fifo_packet(fifo_packet)) {
    uint8_t ii = 0;
    quat[0] = ((int32_t)fifo_packet[0] << 24) | ((int32_t)fifo_packet[1] << 16) | ((int32_t)fifo_packet[2] << 8) | fifo_packet[3];
    quat[1] = ((int32_t)fifo_packet[4] << 24) | ((int32_t)fifo_packet[5] << 16) | ((int32_t)fifo_packet[6] << 8) | fifo_packet[7];
    quat[2] = ((int32_t)fifo_packet[8] << 24) | ((int32_t)fifo_packet[9] << 16) | ((int32_t)fifo_packet[10] << 8) | fifo_packet[11];
    quat[3] = ((int32_t)fifo_packet[12] << 24) | ((int32_t)fifo_packet[13] << 16) | ((int32_t)fifo_packet[14] << 8) | fifo_packet[15];
    ii += 16;
    accel[0] = ((int16_t)fifo_packet[ii + 0] << 8) | fifo_packet[ii + 1];
    accel[1] = ((int16_t)fifo_packet[ii + 2] << 8) | fifo_packet[ii + 3];
    accel[2] = ((int16_t)fifo_packet[ii + 4] << 8) | fifo_packet[ii + 5];
    ii += 6;
    gyro[0] = ((int16_t)fifo_packet[ii + 0] << 8) | fifo_packet[ii + 1];
    gyro[1] = ((int16_t)fifo_packet[ii + 2] << 8) | fifo_packet[ii + 3];
    gyro[2] = ((int16_t)fifo_packet[ii + 4] << 8) | fifo_packet[ii + 5];
  }
}
