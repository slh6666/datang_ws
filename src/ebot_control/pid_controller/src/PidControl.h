#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include <math.h>
#include <algorithm> 

namespace ebot_controller {

class PidControl {
public:
    PidControl(double p, double i, double d, double min, double max) {
      last_error_ = 0.0; int_val_ = 0.0; last_int_val_ = 0.0;
      kp_ = p; ki_ = i; kd_ = d; min_ = min; max_ = max;
    }
    // PidControl(double kp, double ki, double kd, double min, double max) {
    //   last_error_ = 0.0; int_val_ = 0.0; last_int_val_ = 0.0;
    //   kp_ = kp; ki_ = ki; kd_ = kd; min_ = std::min(min,max); max_ = std::max(min,max);
    // }
    void set(double kp,double ki,double kd)
    {
      kp_ = kp;
      ki_ = ki;
      kd_ = kd;
    }

    void setGains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
    void setRange(double min, double max) { min_ = std::min(min,max); max_ = std::max(min,max); }
    void setParams(double kp, double ki, double kd, double min, double max) { setGains(kp,ki,kd); setRange(min,max); }
    void resetIntegrator() { int_val_ = 0.0; last_int_val_ = 0.0; }
    void revertIntegrator() { int_val_ = last_int_val_; } 

    double step(double error, double sample_time) {
      last_int_val_ = int_val_;

      double integral = int_val_ + error * sample_time;
      double derivative = (error - last_error_) / sample_time;

      double y = kp_ * error + ki_ * integral + kd_ * derivative;
      //Output limitation.
      if (y > max_) {
        y = max_;
      } else if (y < min_) {
        y = min_;
      } else {
        int_val_ = integral;
      }
      last_y_ = y;
      last_error_ = error;
      return y;
    }

    double get_last_error(){
      return last_error_;
    }

private:
    double last_error_;
    double int_val_, last_int_val_;
    double last_y_;
    double kp_, ki_, kd_;
    double min_, max_;

};

}

#endif // PIDCONTROL_H

