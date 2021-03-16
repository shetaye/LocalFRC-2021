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
  // Find an open spot in task_queue and queue the task.
  for (int i = 0; i < MAX_TASKS_QUEUED; i++) {
    if (task_queue[i] == NULL) {
      Serial.print("Queued ");
      Serial.println(task->name);
      task->id=i;
      task->status=TASK_QUEUED;
      task_queue[i] = task;
      return i;
    }
  }
  return -1;
}

void Scheduler::kill (Task* task) {
  int tid = task->id;
  if (tasks[tid] != NULL) {
    tasks[tid] = NULL;
    tasks[tid]->status = TASK_KILLED;
    busy = busy & ~tasks[tid]->needs();
  }
}

void Scheduler::run () {
  // Update time
  time = millis();

  // Move eligible tasks from queue to running
  for (int i = 0; i < MAX_TASKS_QUEUED; i++) {
    Task* task = task_queue[i];
    if (task == NULL) continue;

    // If the task is elligible, start running it
    if (!(task->needs() & busy)) { // NAND
      for (int k = 0; k < MAX_TASKS_RUNNING; k++) {
        if (tasks[k] == NULL) {
          // Add to running
          tasks[k] = task;
          // Update status
          task->status = TASK_RUNNING;
          task->started = millis();
          // Remove from queue
          task_queue[i] = NULL;
          // Mark needed subsystems as busy
          busy = busy | task->needs();
          Serial.print("Running ");
          Serial.println(task->name);
          break;
        }
      }
    }
  }
  // Run all tasks in running
  for (int i = 0; i < MAX_TASKS_RUNNING; i++) {
    Task* task = tasks[i];
    if (task != NULL) {
      task->ellapsed = millis() - task->started;
      bool done = task->run(this);
      if (done) {
        // Stop running task, but DON'T DESTROY TASK
        // API consumer could still need the task for
        // status info
        Serial.print("Finished ");
        Serial.println(task->name);
        task->status = TASK_FINISHED;
        tasks[i] = NULL;
        busy = busy & ~task->needs();
      }
    }
  }
}
