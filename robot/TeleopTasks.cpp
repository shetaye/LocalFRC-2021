#include "Tasks.h"
#include "Subsystems.h"
#include "Scheduler.h"
#include "DSState.h"
#include "util.h"

bool Teleop::run(Scheduler* scheduler) {
  if (arcade.status == TASK_CREATED) { scheduler->schedule(&arcade); }
  if (manipulate.status == TASK_CREATED) { scheduler->schedule(&manipulate); }
  return false;
}
void Teleop::kill(Scheduler* scheduler) {
  scheduler->kill(&arcade);
  scheduler->kill(&manipulate);
}

/**
 * Manipulate
 */
bool Manipulate::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  Grabber* grabber = scheduler->get_subsystem<Grabber>(GRABBER_ID);

  float grip_axis = ds->get_axis(2);
  if (grip_axis > 0.5) {
    grabber->open();
  }
  if (grip_axis < -0.5) {
    grabber->close();
  }
  else {
    // Nothing
  }

  // Move elevator around
  //float elevator_axis = ds->get_axis(3);
  bool up = ds->get_button(GamepadButton::B);
  bool down = ds->get_button(GamepadButton::A);
  if (up && !down) {
    elevator->set_direction(ElevatorDirection::Up);
  }
  else if (down && !up) {
    elevator->set_direction(ElevatorDirection::Down);
  } else {
    elevator->set_direction(ElevatorDirection::Hold);
  }
  return false;
}

/**
 * TiltDrive
 */
bool TiltDrive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);

  float lt = map(ds->get_axis(5), 1.0f, -1.0f, 0.0f, 1.0f);
  float rt = map(ds->get_axis(4), 1.0f, -1.0f, 0.0f, 1.0f);


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
bool ArcadeDrive::run(Scheduler* scheduler) {
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  float turn    = map(ds->get_axis(GamepadAxis::LeftX) * -1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
  float forward = map(ds->get_axis(GamepadAxis::LeftY) * +1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

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
    if (turn >= 0.0f) {
      // III
      left = forward + turn;
      right = maxInput;
    }
    else {
      // IV
      left = maxInput + turn;
      right = forward;
    }
  }

  left = clamp(left, -1.0f, 1.0f);
  right = clamp(right, -1.0f, 1.0f);

  scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID)->setPower(left, right);
  return false; // Never done
}
