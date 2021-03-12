#ifndef TASKS_H
#define TASKS_H

#include "Scheduler.h"

class DSPoll : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  DSPoll() { name = "DSPoll"; }
};

class ServoSweep : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  ServoSweep() { name = "Servo Sweep"; }
};

#define ARCADE_SQUARE
class ArcadeDrive : public Task {
  uint8_t needs ();
  bool    run   (Scheduler*);
  public:
  ArcadeDrive() { name = "Arcade Drive"; }
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
