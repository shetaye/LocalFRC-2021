#include <Arduino.h>
#include "Subsystems.h"
#include "pins.h"
#include <Wire.h>
#include "Scheduler.h"
#include "Tasks.h"

#define INITIAL_STATE 0 // 0 for auto, 1 for teleop

volatile bool mpu_int = false;

void mpu_int_trig() {
  mpu_int = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(400000);

  attachInterrupt(digitalPinToInterrupt(P_INTERRUPT), mpu_int_trig, RISING);
}

void loop() {
  Serial.println("Starting...");

  Scheduler scheduler;

  Drivetrain drivetrain;
  DSInterface ds_interface;
  Linetracker linetracker;
  ServoBlock servo_block;
  Elevator elevator(&servo_block);
  Grabber grabber(&servo_block);
  Wrist wrist(&servo_block);
  Ultrasonic ultrasonic;
  Mpu mpu(&mpu_int);

  mpu.init_dmp();

  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  scheduler.register_subsystem(&ultrasonic, ULTRASONIC_ID);
  scheduler.register_subsystem(&mpu, MPU_ID);
  scheduler.register_subsystem(&wrist, WRIST_ID);

  RootTask root;
  scheduler.schedule(&root);

  while(1) { scheduler.run(); }
}
