#include <Arduino.h>
#include "util.h"
#include "Subsystems.h"
#include "pins.h"
#include "DSState.h"
#include "DSProtocol.h"

/**
 * DSInterface
 */
void DSInterface::setup () {
  Serial.begin(57600);
  while (!Serial);
  controlState = ControlState::Auto;
}

bool DSInterface::waitUntilMessageStart (int packetDelay) {
  while (Serial.available() > 0) {
    char recv = Serial.peek();

    if (recv != PACKET_START_CHAR) {
      Serial.read();
    }
    else {
      delay(packetDelay);
      return true;
    }
  }

  return false;
}

void DSInterface::poll () {
  // Hold until message, if no message just cancel
  if (!waitUntilMessageStart(4)) { return; }

  // Scan for valid packets
  if(Serial.available() >= MIN_UART_MESSAGE_LENGTH) {
    // Read into buffer
    int bytes_read = 0;
    while (Serial.available()) {
      if (bytes_read >= sizeof(protocolBuffer)) {
        break;
      }
      protocolBuffer[bytes_read++] = Serial.read();
    }

    // Print current buffer
    /*for (int j = 0; j < bytes_read; j++) {
      Serial.print(protocolBuffer[j]);
      }*/

    // Scan for valid packets
    int i = 0;
    while (i < bytes_read) {
      int bytesRemaining = bytes_read - i;
      char streamType;
      int packet_len = 0;
      if (packet_len = DSProtocol::decodeDSControlPacket(
            &protocolBuffer[i],
            bytesRemaining,
            driverStation.estopped,
            driverStation.enabled,
            driverStation.mode,
            driverStation.switchState
            )) {
        // Received DS packet
      } else if (packet_len = DSProtocol::decodeJoystick1Packet(
            &protocolBuffer[i],
            bytesRemaining,
            driverStation.gamepad1.buttonState,
            driverStation.gamepad1.axis
            )) {
        // Received Joystick 1 packet
      } else if (packet_len = DSProtocol::decodeJoystick2Packet(
            &protocolBuffer[i],
            bytesRemaining,
            driverStation.gamepad2.buttonState,
            driverStation.gamepad2.axis
            )) {
        // Received Joystick 2 packet
      }

      if (packet_len > 0) {
        i += packet_len;
      } else {
        i++;
      }
    }
  }
}

ControlState DSInterface::getControlState() { return controlState; }

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

// "Turn" is -1.0 (left) to +1.0 (right)
// Similarily, "forward" is -1.0 (backward) to +1.0 (forward)
void Drivetrain::arcade(double forward, double turn, bool squareInputs) {

  if (squareInputs) {
    if (forward < 0) { forward *= -forward; }
    else { forward *= forward; }
    if (turn < 0) { turn *= -turn; }
    else { turn *= turn; }
  }

  // (Ignore this) Preserve max input so a hard bank will still cause a significant response
  //double maxInput = max(forward, turn);
  double maxInput = forward;
  double left;
  double right;

  if (forward >= 0) {
    if (turn >= 0) {
      // I
      left = maxInput;
      right = forward - turn;
    }
    else {
      // II
      left = forward + turn;
      right = maxInput;
    }
  }
  else {
    if (turn >= 0) {
      // III
      left = forward + turn;
      right = maxInput;
    }
    else {
      // IV
      left = maxInput;
      right = forward - turn;
    }
  }

  left = clamp(left, -1.0, 1.0);
  right = clamp(right, -1.0, 1.0);

  setPower(left, right);
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
