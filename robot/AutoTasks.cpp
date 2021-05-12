#include "Tasks.h"
#include "Subsystems.h"
#include "Scheduler.h"

/*
 * Parent Tasks
 */

bool Autonomous::run(Scheduler* scheduler) { return true; }
void Autonomous::kill(Scheduler* scheduler) {

}

bool AutoTPBranch::run(Scheduler* scheduler) { return true; }
void AutoTPBranch::kill(Scheduler* scheduler) {

}

bool AutoLineBranch::run(Scheduler* scheduler) { return true; }
void AutoLineBranch::kill(Scheduler* scheduler) {

}

/*
 * Step tasks
 */

bool ForwardUntil::run(Scheduler* scheduler) { return true; }

bool SetGrabber::run(Scheduler* scheduler) { return true; }

bool SetElevator::run(Scheduler* scheduler) { return true; }

bool DumpHex::run(Scheduler* scheduler) { return true; }

bool Turn::run(Scheduler* scheduler) { return true; }

