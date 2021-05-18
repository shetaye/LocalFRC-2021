#include "Tasks.h"
#include "Subsystems.h"
#include "Scheduler.h"
#include <math.h>

/*
 * Parent Tasks
 */

bool Autonomous::run(Scheduler* scheduler) {
  switch(step) {
    case SETUPZeroElevator:
      if (SETUP_zero_elevator.status == TASK_CREATED) { scheduler->schedule(&SETUP_zero_elevator); }
      if (SETUP_zero_elevator.status == TASK_FINISHED) { step = SETUPPrepareElevator; }
      break;
    case SETUPPrepareElevator:
      if (SETUP_raise_elevator.status == TASK_CREATED) { scheduler->schedule(&SETUP_raise_elevator); }
      if (SETUP_raise_elevator.status == TASK_FINISHED) { step = TP1ApproachLine; }
      break;
    case TP1ApproachLine:
      if (TP1_approach.status == TASK_CREATED) { scheduler->schedule(&TP1_approach); }
      if (TP1_approach.status == TASK_FINISHED) { step = TP1Branch; }
      break;
    case TP1Branch:
      if (TP1_approach.line && TP1_line.status == TASK_CREATED) { scheduler->schedule(&TP1_line); }
      if (TP1_approach.obstacle && TP1_tp.status == TASK_CREATED) { scheduler->schedule(&TP1_tp); }
      if (TP1_line.status == TASK_FINISHED || TP1_tp.status == TASK_FINISHED) { step = TP1Approach; }
      break;
    case TP1Approach:
      if (TP1_approach_shelf.status == TASK_CREATED) { scheduler->schedule(&TP1_approach_shelf); }
      if (TP1_approach_shelf.status == TASK_FINISHED) { step = TP1Release; }
      break;
    case TP1Release:
      if (TP1_release_tp.status == TASK_CREATED) { scheduler->schedule(&TP1_release_tp); }
      if (TP1_release_tp.status == TASK_FINISHED) { step = TP2FaceLine; }
      break;
    case TP2FaceLine:
      if (TP2_face_line.status == TASK_CREATED) { scheduler->schedule(&TP2_face_line); }
      if (TP2_face_line.status == TASK_FINISHED) { step = TP2ApproachLine; }
      break;
    case TP2ApproachLine:
      if (TP2_approach_line.status == TASK_CREATED) { scheduler->schedule(&TP2_approach_line); }
      if (TP2_approach_line.status == TASK_FINISHED) { step = TP2Orient; }
      break;
    case TP2Orient:
      if (TP2_orient_on_line.status == TASK_CREATED) { scheduler->schedule(&TP2_orient_on_line); }
      if (TP2_orient_on_line.status == TASK_FINISHED) { step = TP2FaceTP; }
      break;
    case TP2FaceTP:
      if (TP2_face_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_face_tp); }
      if (TP2_face_tp.status == TASK_FINISHED) { step = TP2ApproachTP; }
      break;
    case TP2ApproachTP:
      if (TP2_approach_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_approach_tp); }
      if (TP2_approach_tp.status == TASK_FINISHED) { step = TP2Lower; }
      break;
    case TP2Lower:
      if (TP2_lower_to_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_lower_to_tp); }
      if (TP2_lower_to_tp.status == TASK_FINISHED) { step = TP2Grab; }
      break;
    case TP2Grab:
      if (TP2_grab_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_grab_tp); }
      if (TP2_grab_tp.status == TASK_FINISHED) { step = TP2Raise; }
      break;
    case TP2Raise:
      if (TP2_raise_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_raise_tp); }
      if (TP2_raise_tp.status == TASK_FINISHED) { step = TP2Scooch; }
      break;
    case TP2Scooch:
      if (TP2_schooch.status == TASK_CREATED) { scheduler->schedule(&TP2_schooch); }
      if (TP2_schooch.status == TASK_FINISHED) { step = TP2FaceShelf; }
      break;
    case TP2FaceShelf:
      if (TP2_face_shelf.status == TASK_CREATED) { scheduler->schedule(&TP2_face_shelf); }
      if (TP2_face_shelf.status == TASK_FINISHED) { step = TP2ApproachShelf; }
      break;
    case TP2ApproachShelf:
      if (TP2_approach_shelf.status == TASK_CREATED) { scheduler->schedule(&TP2_approach_shelf); }
      if (TP2_approach_shelf.status == TASK_FINISHED) { step = TP2Release; }
      break;
    case TP2Release:
      if (TP2_release_tp.status == TASK_CREATED) { scheduler->schedule(&TP2_release_tp); }
      if (TP2_release_tp.status == TASK_FINISHED) { return true; }
      break;
  }
  return false;
}
void Autonomous::kill(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  dt->setPower(0., 0.);
  Grabber* grabber = scheduler->get_subsystem<Grabber>(GRABBER_ID);
  grabber->open();
  scheduler->kill(&SETUP_zero_elevator);
  scheduler->kill(&SETUP_raise_elevator);
  scheduler->kill(&TP1_approach);
  scheduler->kill(&TP1_tp);
  scheduler->kill(&TP1_line);
  scheduler->kill(&TP1_approach_shelf);
  scheduler->kill(&TP1_release_tp);
  scheduler->kill(&TP2_face_line);
  scheduler->kill(&TP2_approach_line);
  scheduler->kill(&TP2_orient_on_line);
  scheduler->kill(&TP2_face_tp);
  scheduler->kill(&TP2_approach_tp);
  scheduler->kill(&TP2_lower_to_tp);
  scheduler->kill(&TP2_grab_tp);
  scheduler->kill(&TP2_raise_tp);
  scheduler->kill(&TP2_schooch);
  scheduler->kill(&TP2_face_shelf);
  scheduler->kill(&TP2_approach_shelf);
  scheduler->kill(&TP2_release_tp);
}

bool AutoTPBranch::run(Scheduler* scheduler) {
  switch(step) {
    case TBLower:
      if (lower_to_tp.status == TASK_CREATED) { scheduler->schedule(&lower_to_tp); }
      if (lower_to_tp.status == TASK_FINISHED) { step = TBGrab; }
      break;
    case TBGrab:
      if (grab_tp.status == TASK_CREATED) { scheduler->schedule(&grab_tp); }
      if (grab_tp.status == TASK_FINISHED) { step = TBRaise; }
      break;
    case TBRaise:
      if (raise_tp.status == TASK_CREATED) { scheduler->schedule(&raise_tp); }
      if (raise_tp.status == TASK_FINISHED) { step = TBFaceShelf; }
      break;
    case TBFaceShelf:
      if (face_shelf.status == TASK_CREATED) { scheduler->schedule(&face_shelf); }
      if (face_shelf.status == TASK_FINISHED) { return true; }
      break;
  }
  return false;
}
void AutoTPBranch::kill(Scheduler* scheduler) {
  scheduler->kill(&lower_to_tp);
  scheduler->kill(&grab_tp);
  scheduler->kill(&raise_tp);
  scheduler->kill(&face_shelf);
}

bool AutoLineBranch::run(Scheduler* scheduler) {
  switch(step) {
    case LBOrient:
      if (orient_on_line.status == TASK_CREATED) { scheduler->schedule(&orient_on_line); }
      if (orient_on_line.status == TASK_FINISHED) { step = LBApproachTP; }
      break;
    case LBApproachTP:
      if (approach_tp.status == TASK_CREATED) { scheduler->schedule(&approach_tp); }
      if (approach_tp.status == TASK_FINISHED) { step = LBLower; }
      break;
    case LBLower:
      if (lower_to_tp.status == TASK_CREATED) { scheduler->schedule(&lower_to_tp); }
      if (lower_to_tp.status == TASK_FINISHED) { step = LBGrab; }
      break;
    case LBGrab:
      if (grab_tp.status == TASK_CREATED) { scheduler->schedule(&grab_tp); }
      if (grab_tp.status == TASK_FINISHED) { step = LBRaise; }
      break;
    case LBRaise:
      if (raise_tp.status == TASK_CREATED) { scheduler->schedule(&raise_tp); }
      if (raise_tp.status == TASK_FINISHED) { step = LBFaceWall; }
      break;
    case LBFaceWall:
      if (face_wall.status == TASK_CREATED) { scheduler->schedule(&face_wall); }
      if (face_wall.status == TASK_FINISHED) { step = LBApproachWall; }
      break;
    case LBApproachWall:
      if (approach_wall.status == TASK_CREATED) { scheduler->schedule(&approach_wall); }
      if (approach_wall.status == TASK_FINISHED) { step = LBFaceShelf; }
      break;
    case LBFaceShelf:
      if (face_shelf.status == TASK_CREATED) { scheduler->schedule(&face_shelf); }
      if (face_shelf.status == TASK_FINISHED) { return true; }
      break;
  }
  return false;
}
void AutoLineBranch::kill(Scheduler* scheduler) {
  scheduler->kill(&orient_on_line);
  scheduler->kill(&approach_tp);
  scheduler->kill(&lower_to_tp);
  scheduler->kill(&grab_tp);
  scheduler->kill(&raise_tp);
  scheduler->kill(&face_wall);
  scheduler->kill(&approach_wall);
  scheduler->kill(&face_shelf);
}

/*
 * Step tasks
 */
bool ZeroElevator::run(Scheduler* scheduler) {
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  if (start_time == -1.0) {
    start_time = scheduler->time;
    elevator->set_direction(Down);
  }
  else if (start_time + zero_time <= scheduler->time) {
    elevator->set_direction(Hold);
    elevator->zero();
    return true;
  }
  return false;
}
void ZeroElevator::kill(Scheduler* scheduler) {
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  if (start_time != -1.0 && start_time + zero_time <= scheduler->time) {
    elevator->set_direction(Hold);
    elevator->zero();
  }
}

bool ForwardUntil::run(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  Linetracker* lt = scheduler->get_subsystem<Linetracker>(LINETRACKER_ID);
  Ultrasonic* ul = scheduler->get_subsystem<Ultrasonic>(ULTRASONIC_ID);

  obstacle = ul->distance <= stop_distance;
  line = lt->any();
  hit = obstacle || line;

  Serial.println(line);

  bool should_stop = (stop_at_obstacle && obstacle) || (stop_at_line && line);

  if (should_stop) {
    dt->setPower(0., 0.);
    return true;
  } else {
    dt->setPower(APPROACH_SPEED, APPROACH_SPEED);
    return false;
  }
}
void ForwardUntil::kill(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  dt->setPower(0., 0.);
}

bool SetGrabber::run(Scheduler* scheduler) {
  scheduler->get_subsystem<Grabber>(GRABBER_ID)->set_grip(grip);
  return true;
}

bool SetElevator::run(Scheduler* scheduler) {
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  float error = height - elevator->get_inferred_height();
  if (error <= DAMPING_FACTOR && error >= -DAMPING_FACTOR) {
    elevator->set_direction(Hold);
    return true;
  }
  else if (error < 0) {
    elevator->set_direction(Down);
  }
  else if (error > 0) {
    elevator->set_direction(Up);
  }
  return false;
}
void SetElevator::kill(Scheduler* scheduler) {
  Elevator* elevator = scheduler->get_subsystem<Elevator>(ELEVATOR_ID);
  elevator->set_direction(Hold);
}

bool DumpHex::run(Scheduler* scheduler) { return true; }

bool Turn::run(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  Mpu* mpu = scheduler->get_subsystem<Mpu>(MPU_ID);

  if (!started) {
    float start_angle = mpu->ypr[0];
    end_angle = start_angle + angle;
    if(abs(end_angle) > M_PI) {
      // Correct the wrap around
      float wrap_around = abs(end_angle) - M_PI;
      if( start_angle > 0) { end_angle = -M_PI + wrap_around; }
      else { end_angle = M_PI - wrap_around; }
    }
    if (angle > 0) { dt->setPower(-APPROACH_SPEED, APPROACH_SPEED); }
    else { dt->setPower(APPROACH_SPEED, -APPROACH_SPEED); }
  }
  else {
    float error = mpu->ypr[0];
    if (abs(error) < TURN_ERROR_MARGIN) {
      dt->setPower(0., 0.);
      return true;
    }
  }
  return false;
}
void Turn::kill(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  dt->setPower(0., 0.);
}

bool Forward::run(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  if (start_time == -1) {
    start_time = scheduler->time;
    run_time = (distance / DRIVETRAIN_VELOCITY) * 1000;
    if (distance < 0) {
      dt->setPower(-APPROACH_SPEED, -APPROACH_SPEED);
    }
    else {
      dt->setPower(APPROACH_SPEED, APPROACH_SPEED);
    }
  }
  else if (start_time + run_time <= scheduler->time) {
    dt->setPower(0., 0.);
    return true;
  }
  return false;
}
void Forward::kill(Scheduler* scheduler) {
  Drivetrain* dt = scheduler->get_subsystem<Drivetrain>(DRIVETRAIN_ID);
  dt->setPower(0., 0.);
}

