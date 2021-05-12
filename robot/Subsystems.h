#include <stdint.h>
#include "Scheduler.h"
#include "DSState.h"
#include "DSProtocol.h"
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

class ServoBlock: public Subsystem {
  public:
    void setup     ();
    void set_angle (int servo, uint32_t angle);
    void set_pulse (int servo, int pulse);
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
    void  setup               (ServoBlock* servos);
    void  tick                (float delta);
    void  set_height          (float height);
    float get_height          ();
    float get_error           ();
    float get_inferred_height ();
    void  set_direction (ElevatorDirection direction);
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
    void  setup      (ServoBlock* servos);
    void  set_grip   (float grip);
    float get_grip   ();
    bool  is_gripped ();
    void  open       ();
    void  close      ();
    void  toggle     ();
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
    void setup         ();
    void setPower      (double left, double right);
    void setPower      (int side, double power);
  private:
    void setDirection (int side, int direction);
    void setSpeed     (int side, int speed);
    int speed;
};

class Linetracker: public Subsystem {
  public:
    void setup  ();
    bool left   ();
    bool center ();
    bool right  ();
    bool all    ();
    bool any    ();
};

class DSInterface: public Subsystem {
  public:
    void  setup();
    void  poll();
    float get_axis(GamepadAxis axis);
    bool  get_button(GamepadButton button);
    DSProtocol protocol;
    bool  new_packet;
    bool  enabled;
};

#endif
