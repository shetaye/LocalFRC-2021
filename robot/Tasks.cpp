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
 * ArcadeDrive
 */
uint8_t ArcadeDrive::needs() {
  return DRIVETRAIN;
}
bool ArcadeDrive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  double x = map(ds->driverStation.gamepad1.axis[0] * 1.0, -127.0, 127.0, -1.0, 1.0);
  double y = map(ds->driverStation.gamepad1.axis[1] * -1.0, -127.0, 127.0, -1.0, 1.0);
  scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->arcade(y, x, true);
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
    for (int a = 0; a < 5; a++) {
      Serial.print(a);
      Serial.print(": ");
      Serial.print(axis[a]);
      Serial.print(" ");
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
      } else if (subtask->status == FINISHED) {
        delete subtask;
        subtask = NULL;
        state = AUTO_ORIENT;
      }
      break;
    case AUTO_ORIENT:
      if (subtask == NULL) {
        subtask = new Orient;
        scheduler->schedule(subtask);
      } else if (subtask->status == FINISHED) {
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
