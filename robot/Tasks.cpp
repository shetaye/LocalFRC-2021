#include "Tasks.h"
#include "DSState.h"
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
 * Manipulate
 */
uint8_t Manipulate::needs() {
  return ELEVATOR & GRABBER;
}

bool Manipulate::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  Grabber* grabber = scheduler->get_subsystem<Grabber>(GRABBER_ID);

  int delta = 1000 / MANIPULATE_UPDATE_FREQ;
  if (last_update + delta <= scheduler->time) {
    // Move grabber around
    float grip_axis = ds->get_first_axis(2);
    float current_g = grabber->get_grip();
    float update_magnitude = GRABBER_VELOCITY * (delta / 1000.f);
    if (grip_axis > 0.5f) {
      grabber->set_grip(current_g + update_magnitude);
    }
    if (grip_axis < -0.5f) {
      grabber->set_grip(current_g - update_magnitude);
    }
    last_update = scheduler->time;
    // Update elvator
    float actual_delta = (scheduler->time - last_update) / 1000.;
    //elevator->tick(actual_delta);
  }

  // Move elevator around
  float elevator_axis = ds->get_first_axis(3);
  if (elevator_axis > 0.5) {
    elevator->set_direction(ElevatorDirection::Up);
  }
  else if (elevator_axis < -0.5) {
    elevator->set_direction(ElevatorDirection::Down);
  }
  else {
    elevator->set_direction(ElevatorDirection::Hold);
  }
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

  float lt = map(ds->get_first_axis(5), 1.0f, -1.0f, 0.0f, 1.0f);
  float rt = map(ds->get_first_axis(4), 1.0f, -1.0f, 0.0f, 1.0f);


  float slope = rt - lt;
  float power = (rt + lt) / 2 - 0.5f;
  if (power < 0) power = 0;

  float right = -slope + power;
  float left = slope + power;

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
  float turn    = map(ds->get_first_axis(GamepadAxis::LeftX) * -1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
  float forward = map(ds->get_first_axis(GamepadAxis::LeftY) * +1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

  // Calculate arcade drive -> tank drive mapping
#ifdef ARCADE_SQUARE
  if (forward < 0) { forward *= -forward; }
  else { forward *= forward; }
  if (turn < 0) { turn *= -turn; }
  else { turn *= turn; }
#endif

#ifdef ARCADE_PRESERVE_MAX_INPUT
  float maxInput = max(forward, turn);
#else
  float maxInput = forward;
#endif

  float left;
  float right;

  if (forward >= 0.0f) {
    if (turn >= 0.0f) {
      // I
      left = maxInput + turn;
      right = forward - turn;
    }
    else {
      // II
      left = forward + turn;
      right = maxInput - turn;
    }
  }
  else {
    if (turn >= 0.0f) {
      // III
      left = forward + turn;
      right = maxInput - turn;
    }
    else {
      // IV
      left = maxInput + turn;
      right = forward - turn;
    }
  }

  left = clamp(left, -1.0f, 1.0f);
  right = clamp(right, -1.0f, 1.0f);

  scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->setPower(left, right);
  return false; // Never done
}

uint8_t Drive::needs() {
  return 0; // Needs nothing
}

bool Drive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);

  if (last_press + 500 <= scheduler->time) {
    // Respond to button
    if (ds->get_first_button(4)) {
      if (current_mode == DriveMode::Off) current_mode = DriveMode::Arcade;
      if (current_mode == DriveMode::Arcade) current_mode = DriveMode::Tilt;
      if (current_mode == DriveMode::Tilt) current_mode = DriveMode::Off;
    }
    switch(current_mode) {
      case DriveMode::Off:
        scheduler->kill(&arcade);
        scheduler->kill(&tilt);
        // Briefly violate the needs system
        scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->setPower(0., 0.);
        break;
      case DriveMode::Arcade:
        scheduler->kill(&tilt);
        scheduler->schedule(&arcade);
        break;
      case DriveMode::Tilt:
        scheduler->kill(&arcade);
        scheduler->schedule(&tilt);
        break;
    }
  }
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
    /*for (int a = 0; a < 6; a++) {
      Serial.print(a);
      Serial.print(": ");
      Serial.print(axis[a]);
      Serial.println();
      }*/
    Serial.println(ds->get_first_axis(4));
    /* Line tracker */
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
#define AUTO_MOVE_SPEED 0.4f
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
    drivetrain->setPower(0.0f, 0.0f);
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
