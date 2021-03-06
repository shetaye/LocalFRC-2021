#include "Scheduler.h"
#include <Arduino.h>

/**
 * Subsystems
 */
void Scheduler::register_subsystem (Subsystem* subsystem, int sid) {
  subsystems[sid] = subsystem;
}

/**
 * Tasks
 */
int Scheduler::schedule (Task* task) {
  for (int i = 0; i < MAX_TASKS_RUNNING; i++) {
    if (running[i] == NULL) {
      Serial.print(F("Scheduling "));
      Serial.print(task->id);
      Serial.print(F("@"));
      Serial.println(i);
      task->index=i;
      task->status=TASK_RUNNING;
      task->started=millis();
      running[i] = task;
      return i;
    }
  }
  return -1;
}

void Scheduler::kill (Task* task) {
  if (task->status != TASK_RUNNING) { return; }
  int tidx = task->index;

  Serial.print(F("Killing "));
  Serial.print(task->id);
  Serial.print(F("@"));
  Serial.println(tidx);
  if (running[tidx] != NULL) {
    running[tidx]->kill(this);
    running[tidx]->status = TASK_KILLED;
    running[tidx] = NULL;
  }
}


void Scheduler::run () {
  // Update time
  time = millis();

  // Run all tasks in running
  for (int i = 0; i < MAX_TASKS_RUNNING; i++) {
    Task* task = running[i];
    if (task != NULL) {
      task->ellapsed = millis() - task->started;
      bool done = task->run(this);
      if (done) {
        Serial.print(F("Finished "));
        Serial.print(task->id);
        Serial.print(F("@"));
        Serial.println(task->index);
        task->status = TASK_FINISHED;
        running[i] = NULL;
      }
    }
  }
}
