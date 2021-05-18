#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#include <stdint.h>

#include "Scheduler.h"
#include "pins.h"

#include "Adafruit_PWMServoDriver.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "DSState.h"
#include "DSProtocol.h"

#include "util.h"

#define DRIVETRAIN_ID  1
#define LINETRACKER_ID 2
#define DSINTERFACE_ID 3
#define ELEVATOR_ID    4
#define GRABBER_ID     5
#define FLAP_ID        6
#define WRIST_ID       7
#define ULTRASONIC_ID  8
#define MPU_ID         9

#define DRIVETRAIN  (1 << DRIVETRAIN_ID)
#define LINETRACKER (1 << LINETRACKER_ID)
#define DSINTERFACE (1 << DSINTERFACE_ID)
#define ELEVATOR    (1 << ELEVATOR_ID)
#define GRABBER     (1 << GRABBER_ID)
#define FLAP        (1 << FLAP_ID)
#define WRIST       (1 << WRIST_ID)
#define ULTRASONIC  (1 << ULTRASONIC_ID)
#define MPU         (1 << MPU_ID)

#define SERVOMIN  120
#define SERVOMAX  500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
class ServoBlock: public Subsystem {
  public:
    void set_angle (int servo, uint32_t angle);
    void set_pulse (int servo, int pulse);
    void begin     ();
    Adafruit_PWMServoDriver pwm;
};

#define DRIVER_RADIUS 1.f
#define DRIVER_VELOCITY 1
enum ElevatorDirection {
  Up,
  Down,
  Hold
};
class Elevator: public Subsystem {
  public:
    void  tick                (float);
    void  zero                ();
    float get_inferred_height ();
    void  set_direction (ElevatorDirection direction);
    Elevator(ServoBlock* servo_block) :
      h_hat(0.),
      a_vel(0.),
      dir(Hold),
      sb(servo_block) {}
  private:
    bool  e_correct;
    float h_hat;
    float a_vel;
    ElevatorDirection dir;
    ServoBlock* sb;
};

#define WRIST_MIN_PULSE 120
#define WRIST_MAX_PULSE 500
#define WRIST_UP 1.f
#define WRIST_DOWN -1.f
#define WRIST_HOLD 0.f
class Wrist: public Subsystem {
  public:
    void  up        ();
    void  hold      ();
    void  down      ();
    void  set_angle (float);
    float get_angle ();

    Wrist(ServoBlock* servo_block) :
      a(0.),
      sb(servo_block) {}
  private:
    float a;
    ServoBlock* sb;
};

#define GRABBER_MIN_PULSE 375
#define GRABBER_MAX_PULSE 500
#define GRABBER_GRAB 0.0f
#define GRABBER_OPEN 1.f
class Grabber: public Subsystem {
  public:
    void  set_grip   (float grip);
    float get_grip   ();
    bool  is_gripped ();
    void  open       ();
    void  close      ();
    void  toggle     ();
    Grabber(ServoBlock* servo_block) :
      g(0.),
      sb(servo_block) {}
  private:
    float g;
    ServoBlock* sb;

};

#define FORWARD 0
#define BACKWARD 2
#define BRAKE 3
#define RIGHT 0
#define LEFT 1
#define DEFAULT_SPEED 0
#define MAX_SPEED 255

class Drivetrain: public Subsystem {
  public:
    void setPower      (double left, double right);
    void setPower      (int side, double power);
    void begin         ();
  private:
    void setDirection (int side, int direction);
    void setSpeed     (int side, int speed);
};

class Linetracker: public Subsystem {
  public:
    bool left   ();
    bool center ();
    bool right  ();
    bool all    ();
    bool any    ();
    void begin  ();
};

#define DS_L_BUMPER 64
#define DS_R_BUMPER 128
#define DS_A 1
#define DS_X 8
#define DS_Y 16
#define DS_B 2

class DSInterface: public Subsystem {
  public:
    void  poll();
    float get_axis(GamepadAxis axis);
    bool  get_button(GamepadButton button);
    bool  get_button(int button);
    DSProtocol protocol;
    bool  new_packet;
    bool  enabled;
    DSInterface() :
      protocol(),
      new_packet(false),
      enabled(false) { }
};

#define EMA_ALPHA 0.9f
class Ultrasonic: public Subsystem {
  public:
    void  ping();
    void  begin();
    float distance;
    Ultrasonic() : distance(0.f) { }
};

//
#define MPU_X_GYRO_OFFSET 36
#define MPU_Y_GYRO_OFFSET -34
#define MPU_Z_GYRO_OFFSET -17
#define MPU_X_ACCL_OFFSET 1261
#define MPU_Y_ACCL_OFFSET 1759
#define MPU_Z_ACCL_OFFSET 1255
class Mpu: public Subsystem {
  private:
    // MPU stuff
    MPU6050 mpu;
    uint8_t fifo_buffer[64];
  public:
    bool dmp_ready;
    void poll  ();
    void begin ();

    // Orientation
    Quaternion  q;
    VectorInt16 aa;
    VectorInt16 gy;
    VectorInt16 aaReal;
    VectorInt16 aaWorld;
    VectorFloat gravity;
    float euler[3];
    float ypr[3];

    Mpu() : dmp_ready(false) { }
};

#endif
