#include <stdint.h>
#include "DSState.h"
#include "DSProtocol.h"

#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#define DRIVETRAIN_ID  1
#define LINETRACKER_ID 2
#define DSINTERFACE_ID 3

#define DRIVETRAIN  (1 << DRIVETRAIN_ID)
#define LINETRACKER (1 << LINETRACKER_ID)
#define DSINTERFACE (1 << DSINTERFACE_ID)

class Subsystem {
  public:
    virtual void setup ();
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
    void arcade        (double forward, double turn, bool squareInputs);
    void setPower      (double left, double right);
    void setPower      (int side, double power);
  private:
    void setDirection (int side, int direction);
    void setSpeed     (int side, int speed);
    int speed;
};

#define LEFT 1
#define RIGHT 4
#define CENTER 2

class Linetracker: public Subsystem {
  public:
    void setup  ();
    bool left   ();
    bool center ();
    bool right  ();
    bool all    ();
    bool any    ();
};

enum ControlState { Idle, Auto, Teleop };

class DSInterface: public Subsystem {
  public:
    void setup();
    void poll();
    ControlState getControlState();
    DriverStation driverStation;

  private:
    // Protocol Specific
    char protocolBuffer[64];
    bool waitUntilMessageStart (int packetDelay);

    ControlState controlState;
    double x;
    double y;
};

#endif
