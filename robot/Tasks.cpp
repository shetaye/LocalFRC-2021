#include "Tasks.h"
#include <Arduino.h>
#include "Subsystems.h"
#include "util.h"

/** * DSPoll
*/
uint8_t DSPoll::needs() {
  return DSINTERFACE;
}

bool DSPoll::run(Scheduler* scheduler) {
  scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID)->poll();
  return false; // Never done
}
/**
 * ServoSweep
 */
uint8_t ServoSweep::needs() {
  return SERVOBLOCK;
}

bool ServoSweep::run(Scheduler* scheduler) {
  ServoBlock* servo = scheduler->get_subsystem<ServoBlock>(SERVOBLOCK_ID);
  int angle = (int)(scheduler->time / 50) % 180;
  if (angle > 90) { angle = 180 - angle; }
  servo->set_angle(0, angle * 2);
  return false;
}

/**
 * TiltDrive
 */
uint8_t TiltDrive::needs() {
  return DRIVETRAIN;
}

bool TiltDrive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  double lt = map(ds->driverStation.gamepad1.axis[4] * 1.0, -127.0, 127.0, 0.0, 1.0);
  double rt = map(ds->driverStation.gamepad1.axis[5] * 1.0, -127.0, 127.0, 0.0, 1.0);

  double slope = rt - lt;
  double power = (rt - lt) / 2 - 0.5;

  if (power < 0) power = 0;

  Serial.println(slope);
  Serial.println(power);

  double right = slope + power;
  double left = -slope + power;

  scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->setPower(left, right);

  return false;
}

/**
 * ArcadeDrive
 */
uint8_t ArcadeDrive::needs() {
  return DRIVETRAIN;
}
bool ArcadeDrive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  double turn = map(ds->driverStation.gamepad1.axis[0] * 1.0, -127.0, 127.0, -1.0, 1.0);
  double forward = map(ds->driverStation.gamepad1.axis[1] * -1.0, -127.0, 127.0, -1.0, 1.0);

  // Calculate arcade drive -> tank drive mapping
#ifdef ARCADE_SQUARE
  if (forward < 0) { forward *= -forward; }
  else { forward *= forward; }
  if (turn < 0) { turn *= -turn; }
  else { turn *= turn; }
#endif

#ifdef ARCADE_PRESERVE_MAX_INPUT
  double maxInput = max(forward, turn);
#else
  double maxInput = forward;
#endif

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

  scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->setPower(left, right);
  return false; // Never done
}

/**
 * Logger
 */
uint8_t Logger::needs() {
  return 0; // Only passive, so we don't need anything
}
bool Logger::run(Scheduler* scheduler) {
  if (last_message + 1000 <= scheduler->time) {
    /*
     * DS
     */
    Serial.print("DS: ");
    DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
    signed char* axis = ds->driverStation.gamepad1.axis;
    for (int a = 0; a < 6; a++) {
      Serial.print(a);
      Serial.print(": ");
      Serial.print(axis[a]);
      Serial.println();
    }
    /**
     * Line tracker
     */
    Serial.print(" LT: ");
    Linetracker* linetracker = scheduler->get_subsystem<Linetracker>(LINETRACKER_ID);
    Serial.print(linetracker->left());
    Serial.print(linetracker->center());
    Serial.println(linetracker->right());

    last_message = scheduler->time;
  }
  return false; // Never done
}

/**
 * ==========
 * Autonomous
 * ==========
 */
#define AUTO_MOVE_SPEED 0.4
uint8_t Autonomous::needs() {
  return 0; // Just a coordinator, doesn't touch anything
}
bool Autonomous::run(Scheduler* scheduler) {
  switch(state) {
    case AUTO_FORWARD:
      // Check status of subtask
      if (subtask == NULL) {
        // Start task if not started
        subtask = new ForwardUntilLine;
        scheduler->schedule(subtask);
      } else if (subtask->status == TASK_FINISHED) {
        delete subtask;
        subtask = NULL;
        state = AUTO_ORIENT;
      }
      break;
    case AUTO_ORIENT:
      if (subtask == NULL) {
        subtask = new Orient;
        scheduler->schedule(subtask);
      } else if (subtask->status == TASK_FINISHED) {
        delete subtask;
        state = AUTO_COMPLETE;
      }
      break;
    case AUTO_COMPLETE:
      return true;
      break;
  }
  return false;
}
/**
 * ForwardUntilLine
 */
uint8_t ForwardUntilLine::needs() {
  return DRIVETRAIN;
}
bool ForwardUntilLine::run(Scheduler* scheduler) {
  Linetracker* tracker = scheduler->get_subsystem<Linetracker>(LINETRACKER_ID);
  Drivetrain* drivetrain = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  if (tracker->any()) {
    drivetrain->setPower(0.0, 0.0);
    Serial.println("Stopped");
    return true;
  } else {
    drivetrain->setPower(AUTO_MOVE_SPEED, AUTO_MOVE_SPEED);
    return false;
  }
}
/**
 * Orient
 */
uint8_t Orient::needs() {
  return DRIVETRAIN;
}
bool Orient::run(Scheduler* scheduler) {
  return true;
}
