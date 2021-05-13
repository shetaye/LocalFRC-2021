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
  Scheduler scheduler;

  // Subsystems
  Drivetrain  drivetrain;
  DSInterface  ds_interface;
  Linetracker linetracker;
  ServoBlock servo_block;
  Elevator elevator(&servo_block);
  Grabber grabber(&servo_block);
  // Register subsystems
  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  // Serial.println("Registered Driverstation Interface");
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  // Serial.println("Registered Drivetrain");
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  // Serial.println("Registered Linetracker");
  //servo_block.setup();
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  // Serial.println("Registered Servo Block");
  // Serial.println("Registered subsystems");
  // Serial.println("Initialized");
  RootTask root;
  Serial.println("Before root schedule");
  Serial.println(root.logger.status);
  Serial.println(root.logger.id);

  scheduler.schedule(&root);

  Serial.println("After root schedule");
  Serial.println(root.logger.status);
  Serial.println(root.logger.id);

  while(1) { scheduler.run(); }
}
