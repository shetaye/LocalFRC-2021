#include <Arduino.h>
#include "Subsystems.h"
#include "Scheduler.h"
#include "Tasks.h"

#define INITIAL_STATE 0 // 0 for auto, 1 for teleop

Scheduler scheduler;

void setup() {
  // Subsystems
  Drivetrain  drivetrain;
  DSInterface  ds_interface;
  Linetracker linetracker;
  // Register subsystems
  ds_interface.setup();
  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  Serial.println("Registered Driverstation Interface");
  drivetrain.setup();
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  Serial.println("Registered Drivetrain");
  linetracker.setup();
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  Serial.println("Registered Linetracker");
  Serial.println("Registered subsystems");
  // Start long running tasks
  DSPoll dspoll;
  scheduler.schedule(&dspoll);
  ArcadeDrive arcade_drive;
  scheduler.schedule(&arcade_drive);
  Logger logger;
  scheduler.schedule(&logger);
  Serial.println("Initialized");
}

void loop() {
  scheduler.run();
}
