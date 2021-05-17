#include <Arduino.h>
#include "Subsystems.h"
#include "pins.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"

#include "Scheduler.h"
#include "Tasks.h"

// MPU
MPU6050 mpu_i;
bool mpu_ready;
uint8_t fifo_buffer[64];

// PWM
Adafruit_PWMServoDriver pwm;

// Scheduler
Scheduler scheduler;

// Subsystems
DSInterface ds_interface;
Drivetrain drivetrain;
Linetracker linetracker;
ServoBlock servo_block(&pwm);
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
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  for (int i = 0; i < 64; i++) { fifo_buffer[i] = 0; }

  scheduler.register_subsystem(&ds_interface, DSINTERFACE_ID);
  scheduler.register_subsystem(&drivetrain, DRIVETRAIN_ID);
  scheduler.register_subsystem(&linetracker, LINETRACKER_ID);
  scheduler.register_subsystem(&elevator, ELEVATOR_ID);
  scheduler.register_subsystem(&grabber, GRABBER_ID);
  scheduler.register_subsystem(&ultrasonic, ULTRASONIC_ID);
  scheduler.register_subsystem(&wrist, WRIST_ID);
  scheduler.register_subsystem(&mpu, MPU_ID);

  mpu_ready = mpu_setup(&mpu_i);
  scheduler.schedule(&root);
}

bool mpu_setup(MPU6050* mpu_internal) {
  mpu_internal->initialize();

  uint8_t device_status = mpu_internal->dmpInitialize();

  if (device_status == 0)  {

    mpu_internal->setXGyroOffset(MPU_X_GYRO_OFFSET);
    mpu_internal->setYGyroOffset(MPU_Y_GYRO_OFFSET);
    mpu_internal->setZGyroOffset(MPU_Z_GYRO_OFFSET);
    mpu_internal->setXAccelOffset(MPU_X_ACCL_OFFSET);
    mpu_internal->setYAccelOffset(MPU_Y_ACCL_OFFSET);
    mpu_internal->setZAccelOffset(MPU_Z_ACCL_OFFSET);

    mpu_internal->PrintActiveOffsets();

    mpu_internal->setDMPEnabled(true);

    return true;
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(device_status);
    Serial.println(")");
    return false;
  }
}

void mpu_tick(Mpu* mpu, MPU6050* mpu_internal, uint8_t* fifo_buffer) {
  if (mpu_internal->dmpGetCurrentFIFOPacket(fifo_buffer)) {
    mpu_internal->dmpGetQuaternion(&(mpu->q), fifo_buffer);
    mpu_internal->dmpGetEuler(mpu->euler, &(mpu->q));
    mpu_internal->dmpGetGravity(&(mpu->gravity), &(mpu->q));
    mpu_internal->dmpGetYawPitchRoll(mpu->ypr, &(mpu->q), &(mpu->gravity));
    mpu_internal->dmpGetAccel(&(mpu->aa), fifo_buffer);
    mpu_internal->dmpGetLinearAccel(&(mpu->aaReal), &(mpu->aa), &(mpu->gravity));
    mpu_internal->dmpGetLinearAccelInWorld(&(mpu->aaWorld), &(mpu->aaReal), &(mpu->q));
  }
}

void loop() {
  if(mpu_ready) mpu_tick(&mpu, &mpu_i, fifo_buffer);
  scheduler.run();
}
