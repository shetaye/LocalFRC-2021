#include <Arduino.h>
#include "Subsystems.h"
#include "pins.h"

#include "Wire.h"

#include "Scheduler.h"
#include "Tasks.h"

// Scheduler
Scheduler scheduler;

// Subsystems
DSInterface ds_interface;
Drivetrain drivetrain;
Linetracker linetracker;
ServoBlock servo_block;
Elevator elevator(&servo_block);
Grabber grabber(&servo_block);
Wrist wrist(&servo_block);
Ultrasonic ultrasonic;
Mpu mpu;

// Root task
RootTask root;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(400000);

  servo_block.begin();
  drivetrain.begin();
  ultrasonic.begin();
  mpu.begin();

  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  scheduler.register_subsystem(&ultrasonic, ULTRASONIC_ID);
  scheduler.register_subsystem(&wrist, WRIST_ID);
  scheduler.register_subsystem(&mpu, MPU_ID);

  scheduler.schedule(&root);
}

void loop() {
  scheduler.run();
}
