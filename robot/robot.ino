#include <Arduino.h>
#include "Subsystems.h"
#include "Scheduler.h"
#include "Tasks.h"

#define INITIAL_STATE 0 // 0 for auto, 1 for teleop

Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Subsystems
  Drivetrain  drivetrain;
  DSInterface  ds_interface;
  Linetracker linetracker;
  ServoBlock servo_block;
  Elevator elevator;
  Grabber grabber;
  // Register subsystems
  ds_interface.setup();
  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  // Serial.println("Registered Driverstation Interface");
  drivetrain.setup();
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  // Serial.println("Registered Drivetrain");
  linetracker.setup();
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  // Serial.println("Registered Linetracker");
  servo_block.setup();
  elevator.setup(&servo_block);
  grabber.setup(&servo_block);
  //scheduler.register_subsystem(&servo_block, SERVOBLOCK_ID);
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  // Serial.println("Registered Servo Block");
  // Serial.println("Registered subsystems");
  // Start long running tasks
  //DSPoll dspoll;
  //scheduler.schedule(&dspoll);
  //ArcadeDrive arcade_drive;
  //scheduler.schedule(&arcade_drive);
  //TiltDrive tilt_drive;
  //scheduler.schedule(&tilt_drive);
  //Manipulate manipulate;
  //scheduler.schedule(&manipulate);
  //Drive drive;
  //scheduler.schedule(&drive);
  //Logger logger;
  //scheduler.schedule(&logger);
  // Serial.println("Initialized");
  RootTask root;
  Serial.println(root.logger.status);
  Serial.println(root.logger.id);
  Serial.println(root.logger.started);
  Serial.println(root.logger.ellapsed);

  scheduler.schedule(&root);

  Serial.println(root.logger.status);
  Serial.println(root.logger.id);
  Serial.println(root.logger.started);
  Serial.println(root.logger.ellapsed);

}

void loop() {
  scheduler.run();
}
