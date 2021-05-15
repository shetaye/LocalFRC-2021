#ifndef TASKS_H
#define TASKS_H

#include "Scheduler.h"
#include <Arduino.h>

/* Teleoperated */

#define TELEOP 100
#define ARCADE_DRIVE 101
#define TILT_DRIVE 102
#define MANIPULATE 103

#define ARCADE_SQUARE
class ArcadeDrive : public Task {
  bool run   (Scheduler*);
  void kill  (Scheduler*);

  public:
  ArcadeDrive() : Task(ARCADE_DRIVE) {}
};

class TiltDrive : public Task {

  bool run   (Scheduler*);

  public:
  TiltDrive() : Task(TILT_DRIVE) {}
};

class Manipulate : public Task {

  bool run   (Scheduler*);

  public:
  Manipulate() : Task(MANIPULATE) {}
};

class Teleop : public Task {
  bool run   (Scheduler*);
  void kill  (Scheduler*);

  ArcadeDrive arcade;
  Manipulate  manipulate;

  public:
  Teleop() : Task(TELEOP) {}
};


/* Autonomous */

#define AUTONOMOUS 200
#define AUTO_TP_BRANCH 201
#define AUTO_LINE_BRANCH 202
#define ZERO_ELEVATOR 203
#define FORWARD_UNTIL 204
#define SET_GRABBER 205
#define SET_ELEVATOR 206
#define DUMP_HEX 207
#define TURN 208
#define FORWARD 209

#define DRIVETRAIN_VELOCITY 13.f + 5.f // cm/s
#define DRIVETRAIN_AXLE_LENGTH 12.f // cm
#define LINETRACKER_RADIUS 10.f // cm
// To be calibrated on board
#define LOWER_SHELF_HEIGHT 7.5f // Unit-less
#define UPPER_SHELF_HEIGHT 7.5f + 1.5f // Unit-less
#define APPROACH_SPEED 0.7f // Unit-less
#define SHELF_RANGE 23.f // ""cm"" (not really)
#define WALL_RANGE 14.f
#define TP_RANGE 15.f

class ZeroElevator : public Task {
  double start_time;
  double zero_time;

  bool run  (Scheduler*);
  void kill (Scheduler*);

  public:
  ZeroElevator(double zt) :
    Task(ZERO_ELEVATOR),
    start_time(-1.0),
    zero_time(zt) {}
  ZeroElevator() :
    ZeroElevator(5000) {}
};

class ForwardUntil : public Task {
  bool  stop_at_obstacle;
  bool  stop_at_line;
  float stop_distance;

  bool run  (Scheduler*);
  void kill (Scheduler*);

  public:
  bool hit;
  bool line;
  bool obstacle;

  ForwardUntil(bool l, bool o, float d) :
    stop_at_obstacle(o),
    stop_at_line(l),
    stop_distance(d),
    Task(FORWARD_UNTIL),
    hit(false),
    line(false),
    obstacle(false) {}
};

class SetGrabber : public Task {
  float grip;

  bool run   (Scheduler*);
  void reset ();

  public:
  SetGrabber(float g):
    Task(SET_GRABBER),
    grip(g) {}
};

#define DAMPING_FACTOR 0.1
class SetElevator : public Task {
  float height;

  bool run  (Scheduler*);
  void kill (Scheduler*);

  public:
  SetElevator(float h):
    Task(SET_ELEVATOR),
    height(h) {}
};

class DumpHex : public Task {
  bool run (Scheduler*);

  public:
  DumpHex(): Task(DUMP_HEX) {}
};

class Turn : public Task {
  float angle; // rad
  double run_time;
  double start_time;

  bool run (Scheduler*);
  void kill (Scheduler*);

  public:
  Turn(float r):
    Task(TURN),
    angle(r),
    start_time(-1) {}
};

class Forward : public Task {
  float distance; // cm
  double run_time;
  double start_time;

  bool run (Scheduler*);
  void kill (Scheduler*);

  public:
  Forward(float d):
    Task(FORWARD),
    distance(d),
    start_time(-1) {}
};

enum TPBranchStep {
  TBLower,
  TBGrab,
  TBRaise,
  TBFaceShelf,
};
class AutoTPBranch : public Task {
  bool run  (Scheduler*);
  void kill (Scheduler*);

  TPBranchStep step;

  SetElevator  lower_to_tp;
  SetGrabber   grab_tp;
  SetElevator  raise_tp;
  Turn         face_shelf;

  public:
  AutoTPBranch() :
    Task(AUTO_TP_BRANCH),
    step(TBLower),
    lower_to_tp(0),
    grab_tp(0),
    raise_tp(UPPER_SHELF_HEIGHT),
    face_shelf(M_PI) {}
};

enum LineBranchStep {
  LBOrient,
  LBApproachTP,
  LBLower,
  LBGrab,
  LBRaise,
  LBFaceWall,
  LBApproachWall,
  LBFaceShelf,
};

class AutoLineBranch : public Task {
  bool run  (Scheduler*);
  void kill (Scheduler*);

  LineBranchStep step;

  Forward      orient_on_line;
  ForwardUntil approach_tp;
  SetElevator  lower_to_tp;
  SetGrabber   grab_tp;
  SetElevator  raise_tp;
  Turn         face_wall;
  ForwardUntil approach_wall;
  Turn         face_shelf;

  public:
  AutoLineBranch() :
    Task(AUTO_LINE_BRANCH),
    step(LBOrient),
    orient_on_line(LINETRACKER_RADIUS),
    approach_tp(false, true, TP_RANGE),
    lower_to_tp(0),
    grab_tp(1),
    raise_tp(UPPER_SHELF_HEIGHT),
    face_wall(M_PI),
    approach_wall(false, true, WALL_RANGE),
    face_shelf(-M_PI/2) {}
};


enum AutonomousStep {
  SETUPZeroElevator,
  SETUPPrepareElevator,
  TP1ApproachLine,
  TP1Branch,
  TP1Approach,
  TP1Release,
  TP2FaceLine,
  TP2ApproachLine,
  TP2Orient,
  TP2FaceTP,
  TP2ApproachTP,
  TP2Lower,
  TP2Grab,
  TP2Raise,
  TP2Scooch,
  TP2FaceShelf,
  TP2ApproachShelf,
  TP2Release
};
class Autonomous : public Task {
  bool run   (Scheduler*);
  void kill  (Scheduler*);

  AutonomousStep step;

  ZeroElevator SETUP_zero_elevator;
  SetElevator  SETUP_raise_elevator;
  // TP 1
  ForwardUntil   TP1_approach;
  AutoTPBranch   TP1_tp;
  AutoLineBranch TP1_line;
  ForwardUntil   TP1_approach_shelf;
  SetGrabber     TP1_release_tp;
  // TP 2
  Turn         TP2_face_line;
  ForwardUntil TP2_approach_line;
  Forward      TP2_orient_on_line;
  Turn         TP2_face_tp;
  ForwardUntil TP2_approach_tp;
  SetElevator  TP2_lower_to_tp;
  SetGrabber   TP2_grab_tp;
  SetElevator  TP2_raise_tp;
  Forward      TP2_schooch;
  Turn         TP2_face_shelf;
  ForwardUntil TP2_approach_shelf;
  SetGrabber   TP2_release_tp;

  public:
  Autonomous():
    Task(AUTONOMOUS),
    step(SETUPZeroElevator),
    SETUP_zero_elevator(2000),
    SETUP_raise_elevator(4),
    TP1_approach(true, true, TP_RANGE),
    TP1_tp(),
    TP1_line(),
    TP1_approach_shelf(false, true, SHELF_RANGE),
    TP1_release_tp(0),
    TP2_face_line(-M_PI),
    TP2_approach_line(true, false, 0),
    TP2_orient_on_line(LINETRACKER_RADIUS),
    TP2_face_tp(M_PI/2),
    TP2_approach_tp(false, true, TP_RANGE),
    TP2_lower_to_tp(0),
    TP2_grab_tp(1),
    TP2_raise_tp(LOWER_SHELF_HEIGHT),
    TP2_schooch(3.),
    TP2_face_shelf(M_PI/2),
    TP2_approach_shelf(false, true, SHELF_RANGE),
    TP2_release_tp(0) {}
};

/* Always running */

#define ROOT_TASK 0
#define POLL 1
#define LOGGER 2

class Poll : public Task {
  double last_poll;

  bool run (Scheduler*);

  public:
  Poll() :
    Task(POLL),
    last_poll(0.0) {}
};

class Logger : public Task {
  double last_message;

  bool run (Scheduler*);

  public:
  Logger() :
    Task(LOGGER),
    last_message(0) {}
};

enum RobotMode {
  Teleoperated,
  Auto,
  Disabled
};

class RootTask : public Task {
  double  last_pressed;
  RobotMode mode;

  public:
  bool    run   (Scheduler*);
  void    kill  (Scheduler*);

  Autonomous   autonomous;
  Teleop       teleop;
  Poll         poll;
  Logger       logger;

  RootTask() :
    Task(ROOT_TASK),
    last_pressed(0),
    mode(Disabled) {}
};

#endif
