#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include "Subsystems.h"
#include <Arduino.h>

#define MAX_TASKS_RUNNING 16
#define MAX_TASKS_QUEUED  32

#define QUEUED 0
#define RUNNING 1
#define FINISHED 2
#define KILLED 3

// TODO: Make task queue an ACTUAl queue
class Task; // Forward decl
class Scheduler {
  public:
    // Tasks
    int  schedule (Task*);
    void kill     (Task*);
    void run      ();
    // Subsystems
    void       register_subsystem (Subsystem*, int);
    template <class T>
      T* get_subsystem            (int);
    // Utility
    double time;
  private:
    Task*      tasks[MAX_TASKS_RUNNING];
    Task*      task_queue[MAX_TASKS_QUEUED];
    Subsystem* subsystems[8];
    uint8_t    busy;
};

class Task {
  public:
    int    status;
    int    id;
    double started;
    double ellapsed;
    char*  name;
    // Virtual members
    virtual uint8_t needs ();
    virtual bool    run   (Scheduler*);
};

template <class T> T* Scheduler::get_subsystem (int sid) {
  return static_cast<T*>(subsystems[sid]);
}
#endif
