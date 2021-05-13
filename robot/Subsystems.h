#include <stdint.h>
#include "Scheduler.h"
#include "DSState.h"
#include "DSProtocol.h"
#include "pins.h"
#include "Adafruit_PWMServoDriver.h"

#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#define DRIVETRAIN_ID  1
#define LINETRACKER_ID 2
#define DSINTERFACE_ID 3
#define ELEVATOR_ID    4
#define GRABBER_ID     5
#define FLAP_ID        6

#define DRIVETRAIN  (1 << DRIVETRAIN_ID)
#define LINETRACKER (1 << LINETRACKER_ID)
#define DSINTERFACE (1 << DSINTERFACE_ID)
#define ELEVATOR    (1 << ELEVATOR_ID)
#define GRABBER     (1 << GRABBER_ID)
#define FLAP        (1 << FLAP_ID)

#define SERVOMIN  120
#define SERVOMAX  500
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
class ServoBlock: public Subsystem {
  public:
    void set_angle (int servo, uint32_t angle);
    void set_pulse (int servo, int pulse);
    ServoBlock() :
      pwm() {
        pwm.begin();
        pwm.setPWMFreq(SERVO_FREQ);
        delay(10);
      }
  private:
    Adafruit_PWMServoDriver pwm;
};

#define ELEVATOR_MIN_ANGLE 0
#define ELEVATOR_MAX_ANGLE 180
#define DRIVER_RADIUS 1
#define DRIVER_VELOCITY 1
#define DAMPING_FACTOR 0.1
enum ElevatorDirection {
  Up,
  Down,
  Hold
};
class Elevator: public Subsystem {
  public:
    void  tick                (float delta);
    void  set_height          (float height);
    float get_height          ();
    float get_error           ();
    float get_inferred_height ();
    void  set_direction (ElevatorDirection direction);
    Elevator(ServoBlock* servo_block) :
      h(0.),
      h_hat(0.),
      error(0.),
      a_vel(0.),
      dir(Hold),
      sb(servo_block) {}
  private:
    float h;
    float h_hat;
    float error;
    float a_vel;
    ElevatorDirection dir;
    ServoBlock* sb;
};

#define GRABBER_MIN_ANGLE 30
#define GRABBER_MAX_ANGLE 120
#define GRABBER_GRAB 0.4f
#define GRABBER_OPEN 0.8f
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
#define BACKWARD 1
#define BRAKE 3
#define RIGHT 0
#define LEFT 1
#define DEFAULT_SPEED 0
#define MAX_SPEED 255

class Drivetrain: public Subsystem {
  public:
    void setPower      (double left, double right);
    void setPower      (int side, double power);
    Drivetrain() {
      // Init speed
      pinMode(P_LEFT_SPEED, OUTPUT);
      pinMode(P_RIGHT_SPEED, OUTPUT);
      pinMode(P_LEFT_1, OUTPUT);
      pinMode(P_LEFT_2, OUTPUT);
      pinMode(P_RIGHT_1, OUTPUT);
      pinMode(P_RIGHT_2, OUTPUT);
      setPower(0.0, 0.0);
    }
  private:
    void setDirection (int side, int direction);
    void setSpeed     (int side, int speed);
    //int speed;
};

class Linetracker: public Subsystem {
  public:
    bool left   ();
    bool center ();
    bool right  ();
    bool all    ();
    bool any    ();
    Linetracker() {
      pinMode(P_LEFT, INPUT);
      pinMode(P_CENTER, INPUT);
      pinMode(P_RIGHT, INPUT);
    }
};

class DSInterface: public Subsystem {
  public:
    void  poll();
    float get_axis(GamepadAxis axis);
    bool  get_button(GamepadButton button);
    DSProtocol protocol;
    bool  new_packet;
    bool  enabled;
    DSInterface() :
      protocol(),
      new_packet(false),
      enabled(false) { }
};

#endif
