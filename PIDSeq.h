//
// Title: Arduino PID Sequencer
// Author: Orange Cat
// Date: 23-10-2015
//
// Allows setting up a sequence of PID operations (i.e. PID paramaters (setpoint, Kp, Ki, Kd) plus ramp times and hold times).
// When the setpoint and hold time is reached (within epsilon of being reached) it will move onto the next operation in the sequence.
// Ideal for controling ovens and the relatively complex profiles associated with annealing, reflow, etc.
//
// License:
//   This firmware is released under the Creative Commons Attribution-ShareAlike 4.0
//   International license.
//     http://creativecommons.org/licenses/by-sa/4.0/
//

#ifndef PIDSEQ_H
#define PIDSEQ_H

#include <PID_v1.h>

static const double kPIDOpDefaultPercentEpsilon = 0.02;    // distance away from setpoint (0.02 = 2 percent) where we assume we have reached the setpoint

class PIDSeq;

class PIDOp {
  friend class PIDSeq;

  public:
    void begin(double setpoint, double Kp, double Ki, double Kd);

    // setting additional parameters for an operation:
    void setRampTime(unsigned long sec) { ramp_sec_ = sec; }                  // default is 0
    void setHoldTime(unsigned long sec) { hold_sec_ = sec; }                  // default is 0
    void setReverse(bool reverse) { dir_ = (reverse)? REVERSE: DIRECT; }      // default is false
    bool isReverse() const { return (dir_ == REVERSE); }
    void setControlLimits(double min, double max) { min_ = min; max_ = max; } // default is 0.0, 255.0
    void setEpsilon(double epsilon) { epsilon_ = epsilon; }                   // allowed error, default is kPIDOpDefaultPercentEpsilon * setpoint

    // just for associating a name with the op (great for displaying a status)
    void setName(const char* name) { name_ = name; flash_name_ = NULL; }      // default is ""
    void setName(const __FlashStringHelper* name) { flash_name_ = name; name_ = NULL; }    // default is NULL
    const char* name() const { return name_; }
    const __FlashStringHelper* flashName() const { return flash_name_; }
    
  public:
    PIDOp();

  private:
    double setpoint_;
    double Kp_;
    double Ki_;
    double Kd_;
 
    double epsilon_;
    int dir_;
    double ramp_sec_;
    double hold_sec_;
    double min_;
    double max_;
    const char* name_;
    const __FlashStringHelper* flash_name_;
    
    PIDOp* next_;
};

class PIDSeq
{
  public:
    void begin();
    void setSampleTime(unsigned long sample_time_ms) { sample_time_ms_ = sample_time_ms; }
    
    // a method for adding another PID operation to our sequence
    void addOp(PIDOp& op);

    void start(double process);     // called to initiate the control sequence
    double control(double process); // called each time through loop() takes the current process variable and returns the control variable
    void abort();                   // called to abort the control sequence

    double curSetpoint() const { return cur_setpoint_; }          // returns the current setpoint
    bool isComplete() const { return is_complete_; }              // true control sequence is complete (or was aborted)
    bool wasStarted() const { return was_started_; }              // true if control sequence was started

    // return the current sequences operation name and whether the operation is set to reverse respectively:
    String curOpName() const { if (cur_op_ != NULL) return cur_op_->name(); else return ""; }
    const __FlashStringHelper* curOpFlashName() const { if (cur_op_ != NULL) return cur_op_->flashName(); else return NULL; }
    bool curOpIsReverse() const { if (cur_op_ != NULL) return cur_op_->isReverse(); else return false; }
  
  public:
      // generally not needed, but returns a pointer to the current op
      PIDOp* curOp() const { return cur_op_; }

  public:
    PIDSeq();

  private:
    void configPIDForCurrentOp();
    bool isSetpointReached() const;

  private:
    PID pid_;
    unsigned long sample_time_ms_;

    PIDOp* first_op_;
    PIDOp* last_op_;
    PIDOp* cur_op_;
    bool setpoint_reached_;

    unsigned long hold_time_start_;
    double ramp_start_;
    double ramp_slope_;
    unsigned long ramp_time_start_;

    double cur_setpoint_;
    double process_;
    double control_;

    bool is_complete_;
    bool was_started_;
};

#endif // PIDSEQ_H


