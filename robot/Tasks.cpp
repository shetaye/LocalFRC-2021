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
  if(poll.status != TASK_RUNNING) { scheduler->schedule(&poll); }
  if(logger.status != TASK_RUNNING) { scheduler->schedule(&logger); }

  DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
  // Could be more concise if I looked at the tasks instead of maintaining
  // a seperate state i.e. teleop.status == TASK_RUNNING vs mode == Teleoperated
  if (mode == Disabled) {
    if (ds->enabled) {
      Serial.println(F("Enabled"));
      mode = Teleoperated;
      scheduler->schedule(&teleop);
    }
  }
  if (mode == Teleoperated) {
    if (!ds->enabled) {
      Serial.println(F("Disabled"));
      mode = Disabled;
      scheduler->kill(&teleop);
    }
    else if (last_pressed + 1000 <= scheduler->time && ds->get_button(DS_Y)) {
      Serial.println(F("Switching to auto"));
      mode = Auto;
      scheduler->kill(&teleop);
      scheduler->schedule(&autonomous);
      last_pressed = scheduler->time;
    }
  }
  if (mode == Auto) {
    if (!ds->enabled) {
      Serial.println(F("Disabled"));
      mode = Disabled;
      scheduler->kill(&autonomous);
    }
    else if (last_pressed + 1000 <= scheduler->time && ds->get_button(DS_Y)) {
      Serial.println(F("Switching to teleop"));
      mode = Teleoperated;
      scheduler->kill(&autonomous);
      scheduler->schedule(&teleop);
      last_pressed = scheduler->time;
    }
  }
  return false;
}
void RootTask::kill(Scheduler* scheduler) {
  scheduler->kill(&poll);
  scheduler->kill(&logger);
  if (teleop.status == TASK_RUNNING) { scheduler->kill(&teleop); }
  if (autonomous.status == TASK_RUNNING) { scheduler->kill(&autonomous); }
}

/**
 * Poll
 */
bool Poll::run(Scheduler* scheduler) {
  double d = (scheduler->time - last_poll) / 1000;
  scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID)->poll();
  scheduler->get_subsystem<Elevator>(ELEVATOR_ID)->tick((float)d);
  scheduler->get_subsystem<Ultrasonic>(ULTRASONIC_ID)->ping();
  last_poll = scheduler->time;
  return false;
}

/**
 * Logger
 */
bool Logger::run(Scheduler* scheduler) {
  if (last_message + 1000 <= scheduler->time) {
    /*DSInterface* ds = scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID);
      Linetracker* linetracker = scheduler->get_subsystem<Linetracker>(LINETRACKER_ID);
      Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
      Ultrasonic* ultrasonic = scheduler->get_subsystem<Ultrasonic>(ULTRASONIC_ID);*/
    Mpu* mpu = scheduler->get_subsystem<Mpu>(MPU_ID);
    Serial.println(mpu->ypr[0]);
    //Serial.print("H ");
    //Serial.println(elevator->get_inferred_height());
    //Serial.print("U ");
    //Serial.println(ultrasonic->distance);
    //Serial.println(scheduler->get_subsystem<DSInterface>(DSINTERFACE_ID)->protocol.getStatus().gamepad.buttonState);
    //Serial.println(linetracker->any());
    last_message = scheduler->time;
  }
  return false;
}

