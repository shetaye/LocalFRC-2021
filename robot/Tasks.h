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
#define FORWARD_UNTIL 203
#define SET_GRABBER 204
#define SET_ELEVATOR 205
#define DUMP_HEX 206
#define TURN 207

class AutoTPBranch : public Task {

  bool run   (Scheduler*);
  void kill  (Scheduler*);

  Task* current_task;

  public:
  AutoTPBranch() : Task(AUTO_TP_BRANCH) {}
};

class AutoLineBranch : public Task {

  bool    run   (Scheduler*);
  void    kill  (Scheduler*);

  Task* current_task;

  public:
  AutoLineBranch() : Task(AUTO_LINE_BRANCH) {}
};

class ForwardUntil : public Task {
  bool stop_at_obstacle;
  bool stop_at_line;

  bool hit;
  bool line;
  bool obstacle;

  bool run   (Scheduler*);

  public:
  ForwardUntil(bool o, bool l) :
    stop_at_obstacle(o),
    stop_at_line(l),
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

class SetElevator : public Task {
  float height;

  bool run   (Scheduler*);
  void reset ();

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
  float angle;

  bool run (Scheduler*);

  public:
  Turn(float a):
    Task(TURN),
    angle(a) {}
};

class Autonomous : public Task {

  bool    run   (Scheduler*);
  void    kill  (Scheduler*);

  Task* current_task;

  public:
  Autonomous(): Task(AUTONOMOUS) {}
};

/* Always running */

#define ROOT_TASK 0
#define DS_POLL 1
#define LOGGER 2

class DSPoll : public Task {
  bool    run   (Scheduler*);

  public:
  DSPoll() : Task(DS_POLL) {
  }
};


class Logger : public Task {
  double last_message;

  bool    run   (Scheduler*);

  public:
  Logger() :
    Task(LOGGER),
    last_message(0) {
    }
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

  Autonomous  autonomous;
  Teleop      teleop;
  //DSPoll      dspoll;
  Logger      logger;

  RootTask() :
    Task(ROOT_TASK),
    last_pressed(0),
    mode(Disabled) {}
};

#endif
