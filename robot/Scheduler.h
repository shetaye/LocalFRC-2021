#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>
#include <Arduino.h>

#define MAX_TASKS_RUNNING 16

#define TASK_QUEUED 0
#define TASK_RUNNING 1
#define TASK_FINISHED 2
#define TASK_KILLED 3
#define TASK_CREATED 4

class Task; // Forward decl
class Subsystem; // Forward decl
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
    void init();

    Scheduler() :
      time(0.0) {
        // NULL initialize running
        for (int i = 0; i < MAX_TASKS_RUNNING; i++) {
          running[i] = NULL;
        }
        // NULL initialzie subsystms
        for (int i = 0; i < 8; i++) {
          subsystems[i] = NULL;
        }
      }
  private:
    Task* running[MAX_TASKS_RUNNING];
    Subsystem* subsystems[8];
};

class Task {
  public:
    int    status;
    int    index;
    int    id;
    double started;
    double ellapsed;
    Task(int i) :
      status(TASK_CREATED),
      index(-1),
      started(0),
      ellapsed(0),
      id(i) {}
    // Virtual members
    virtual bool    run   (Scheduler*) { return true; }
    virtual void    kill  (Scheduler*) {};
};

class Subsystem {
  //public:
  // Virtual members
  //virtual void setup ();
};


template <class T> T* Scheduler::get_subsystem (int sid) {
  return static_cast<T*>(subsystems[sid]);
}
#endif
