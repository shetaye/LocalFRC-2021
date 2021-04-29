#ifndef TASKS_H
#define TASKS_H

#include "Scheduler.h"

class DSPoll : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  DSPoll() { name = "DSPoll"; }
};

#define GRABBER_VELOCITY 1.f
#define MANIPULATE_UPDATE_FREQ 100
class Manipulate : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);

  double  last_update;
  public:
  Manipulate() {
    last_update = 0;
    name = "Manipulate";
  }
};

class TiltDrive : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  TiltDrive() { name = "Tilt Drive"; }
};

#define ARCADE_SQUARE
//#define ARCADE_PRESERVE_MAX_INPUT
class ArcadeDrive : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  ArcadeDrive() { name = "Arcade Drive"; }
};

enum DriveMode {
  Off    = 0,
  Arcade = 1,
  Tilt   = 2
};
class Drive : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);

  double      last_press;
  DriveMode   current_mode;
  TiltDrive   tilt;
  ArcadeDrive arcade;
  public:
  Drive() { name = "Drive"; current_mode = DriveMode::Off; }
};

class Logger : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);

  double  last_message;
  public:
  Logger() {
    last_message = 0;
    name = "Logger";
  }
};

/**
 * Autonomous
 */
#define AUTO_FORWARD 1
#define AUTO_ORIENT 2
#define AUTO_COMPLETE 3
class Autonomous : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);

  int   state;
  Task* subtask;
  public:
  Autonomous() {
    state = AUTO_FORWARD;
    subtask = NULL;
    name = "Autonomous";
  }
};
class ForwardUntilLine : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  ForwardUntilLine() { name = "ForwardUntilLine"; }
};
class Orient : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  Orient() { name = "Orient"; }
};
#endif
