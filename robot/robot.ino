#include <Arduino.h>
#include "Subsystems.h"
#include "Scheduler.h"
#include "Tasks.h"

#define INITIAL_STATE 0 // 0 for auto, 1 for teleop

void setup() {
  Serial.begin(115200);
  while (!Serial);
}

void loop() {
  Serial.println("ABC");
  Scheduler scheduler;

  Drivetrain  drivetrain;
  DSInterface  ds_interface;
  Linetracker linetracker;
  ServoBlock servo_block;
  Elevator elevator(&servo_block);
  Grabber grabber(&servo_block);
  Wrist wrist(&servo_block);
  Ultrasonic ultrasonic;

  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  scheduler.register_subsystem(&ultrasonic, ULTRASONIC_ID);
  scheduler.register_subsystem(&wrist, WRIST_ID);

  RootTask root;
  scheduler.schedule(&root);

  while(1) { scheduler.run(); }
}
