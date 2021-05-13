#include "Tasks.h"
#include "DSState.h"
#include <Arduino.h>
#include "Subsystems.h"
#include "Scheduler.h"
#include "util.h"

/**
 * RootTask
 */
bool RootTask::run(Scheduler* scheduler) {
  Serial.println("In RootTask::run");
  Serial.println(logger.status);
  Serial.println(logger.id);

  if(dspoll.status == TASK_CREATED) { scheduler->schedule(&dspoll); }
  if(logger.status == TASK_CREATED) { scheduler->schedule(&logger); }

  return true;
  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  // Could be more concise if I looked at the tasks instead of maintaining
  // a seperate state i.e. teleop.status == TASK_RUNNING vs mode == Teleoperated
  if (mode == Disabled) {
    if (ds->enabled) {
      Serial.println("Enabled");
      mode = Teleoperated;
      scheduler->schedule(&teleop);
    }
  }
  if (mode == Teleoperated) {
    if (!ds->enabled) {
      Serial.println("Disabled");
      mode = Disabled;
      scheduler->kill(&teleop);
    }
    else if (last_pressed + 1000 <= scheduler->time && ds->get_button(GamepadButton::Y)) {
      Serial.println("Switching to auto");
      mode = Auto;
      scheduler->kill(&teleop);
      scheduler->schedule(&autonomous);
    }
  }
  if (mode == Auto) {
    if (!ds->enabled) {
      Serial.println("Disabled");
      mode = Disabled;
      scheduler->kill(&autonomous);
    }
    else if (last_pressed + 1000 <= scheduler->time && ds->get_button(GamepadButton::Y)) {
      Serial.println("Switching to teleop");
      mode = Teleoperated;
      scheduler->kill(&autonomous);
      scheduler->schedule(&teleop);
    }
  }
  return false;
}
void RootTask::kill(Scheduler* scheduler) {
  //scheduler->kill(&dspoll);
  scheduler->kill(&logger);
  if (teleop.status == TASK_RUNNING) { scheduler->kill(&teleop); }
  if (autonomous.status == TASK_RUNNING) { scheduler->kill(&autonomous); }
}

/**
 * DSPoll
 */
bool DSPoll::run(Scheduler* scheduler) {
  //scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID)->poll();
  return false;
}

/**
 * Logger
 */
bool Logger::run(Scheduler* scheduler) {
  if (last_message + 1000 <= scheduler->time) {
    Serial.print("LT: ");
    Linetracker* linetracker = scheduler->get_subsystem<Linetracker>(LINETRACKER_ID);
    Serial.print(linetracker->left());
    Serial.print(linetracker->center());
    Serial.println(linetracker->right());
    last_message = scheduler->time;
  }
  return false;
}

