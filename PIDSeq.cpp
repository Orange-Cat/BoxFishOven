//
// Title: Arduino PID Sequencer
// Author: Orange Cat
// Date: 2015-10-24
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//
#include "Arduino.h"
#include "PIDSeq.h"


PIDOp::PIDOp()
  :setpoint_(0.0),
  Kp_(0.0),
  Ki_(0.0),
  Kd_(0.0),
  epsilon_(0.0),
  dir_(DIRECT),
  ramp_sec_(0),
  hold_sec_(0),
  min_(0),
  max_(0),
  name_(""),
  flash_name_(NULL),
  next_(NULL)
{
  return;
}

void PIDOp::begin(double setpoint, double Kp, double Ki, double Kd)
{
  setpoint_ = setpoint;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  epsilon_ = setpoint * kPIDOpDefaultPercentEpsilon;
  dir_ = DIRECT;
  ramp_sec_ = 0;
  hold_sec_ = 0;
  min_ = 0;
  max_ = 255;
  name_ = "";
  flash_name_ = NULL;
  next_ = NULL;
}

PIDSeq::PIDSeq()
  :pid_(&process_, &control_, &cur_setpoint_, 1.0, 0.0, 0.0, DIRECT),
  sample_time_ms_(0),
  first_op_(NULL),
  last_op_(NULL),
  cur_op_(NULL),
  setpoint_reached_(false),
  hold_time_start_(0),
  ramp_start_(0.0),
  ramp_slope_(0.0),
  ramp_time_start_(0),
  cur_setpoint_(0.0),
  process_(0.0),
  control_(0.0),
  is_complete_(false),
  was_started_(false)
{
  return;
}

void PIDSeq::begin()
{
  abort();
  sample_time_ms_ = 100;
  first_op_ = NULL;
  last_op_ = NULL;
  cur_op_ = NULL;
  setpoint_reached_ = false;
  hold_time_start_ = 0;
  ramp_start_ = 0.0;
  ramp_slope_ = 0.0;
  ramp_time_start_ = 0;
  cur_setpoint_ = 0.0;
  process_ = 0.0;
  control_ = 0.0;
  is_complete_ = true;
  was_started_ = false;
  pid_ = PID(&process_, &control_, &cur_setpoint_, 1.0, 0.0, 0.0, DIRECT);
}

void PIDSeq::addOp(PIDOp& op)
{
  if (first_op_ == NULL) {
    first_op_ = &op;
    cur_op_ = &op;
    last_op_ = &op;
  }
  else {
    last_op_->next_ = &op;
    last_op_ = &op;
  }
  op.next_ = NULL;
}

void PIDSeq::start(double process)
{
  // called to start the control sequence
  if (last_op_ == NULL) {
    return;
  }

  process_ = process;
  cur_op_ = first_op_;
  setpoint_reached_ = false;
  is_complete_ = false;
  was_started_ = true;
  pid_.SetSampleTime(sample_time_ms_);
  configPIDForCurrentOp();
  pid_.SetMode(AUTOMATIC);
}

double PIDSeq::control(double process)
{
  // when in operation, should be called each time through loop()
  // takes the current process variable and returns the current contol variable

  if (cur_op_ == NULL || is_complete_) {
    // if we are complete, just return the minimum control variable from the last op
    return last_op_->min_;
  }

  process_ = process;

  if (!setpoint_reached_ && isSetpointReached()) {
    setpoint_reached_ = true;
    hold_time_start_ = millis();
  }

  if (setpoint_reached_) {
    //  the setpoint has been reached, however we only move to the next sequence after we've passed the hold time at this setpoint
    unsigned long time_in_hold = (millis() - hold_time_start_) / 1000;

    if (time_in_hold >= cur_op_->hold_sec_) {
      // we're done with this sequence

      if (cur_op_->next_ == NULL) {
        // we are completely done
        is_complete_ = true;
        pid_.SetMode(MANUAL);
        return cur_op_->min_;
      }
      cur_op_ = cur_op_->next_;
      configPIDForCurrentOp();
      setpoint_reached_ = false;
    }
  }

  // PID's interface for Compute() accesses input variables setpoint_ and process_, and output variable control_ through pointers passed to the constructor
  bool did_calc = pid_.Compute();

  if (did_calc) {
    // if we have a ramp time then cur_setpoint_ may be below the setpoint, we need to adjust the current setpoint from time to time
    if ((ramp_slope_ > 0.0 && cur_setpoint_ < cur_op_->setpoint_) || (ramp_slope_ < 0.0 && cur_setpoint_ > cur_op_->setpoint_))  {
      unsigned long time_in_ramp = (millis() - ramp_time_start_) / 1000;
      cur_setpoint_ = ramp_start_ + (double)time_in_ramp * ramp_slope_;
      if ((ramp_slope_ > 0.0 && cur_setpoint_ > cur_op_->setpoint_) || (ramp_slope_ < 0.0 && cur_setpoint_ < cur_op_->setpoint_)) {
        cur_setpoint_ = cur_op_->setpoint_;
      }
    }

  }
  return control_;
}

void PIDSeq::abort()
{
  // called to abort the control sequence
  pid_.SetMode(MANUAL);
  is_complete_ = true;
}

void PIDSeq::configPIDForCurrentOp()
{
  pid_.SetTunings(cur_op_->Kp_, cur_op_->Ki_, cur_op_->Kd_);
  pid_.SetControllerDirection(cur_op_->dir_);
  pid_.SetOutputLimits(cur_op_->min_, cur_op_->max_);
  if (cur_op_->ramp_sec_ * 1000uL <= sample_time_ms_) {
    cur_setpoint_ = cur_op_->setpoint_;
  }
  else {
    // we are ramping, set the starting point for the ramp and detemrine the slope
    if (cur_op_ != first_op_) {
      ramp_start_ = cur_setpoint_;  // use previous setpoint as ramp start
    }
    else {
      ramp_start_ = process_;   // use existing process variable as ramp start
      cur_setpoint_ = ramp_start_;
    }
    ramp_slope_ = (cur_op_->setpoint_ - process_) / cur_op_->ramp_sec_;
    ramp_time_start_ = millis();
  }
}

bool PIDSeq::isSetpointReached() const
{
  // returns true if ramp is complete and we are within epsilon of our target (i.e. final) setpoint

  if ((ramp_slope_ > 0.0 && cur_setpoint_ < cur_op_->setpoint_) || (ramp_slope_ < 0.0 && cur_setpoint_ > cur_op_->setpoint_)) {
    // we are still ramping
    return false;
  }

  return (cur_op_->dir_ == DIRECT && process_ >= (cur_op_->setpoint_ - cur_op_->epsilon_))
         || (cur_op_->dir_ == REVERSE && process_ <= (cur_op_->setpoint_ + cur_op_->epsilon_));
}


