#ifndef PID_H
#define PID_H

#include <vector>

#include "Twiddle.h"

class PID {
public:

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tuning);

  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateError(double cte, bool& reset);



private:
  void Reset();
  Twiddle twiddle_;
  bool first_update_ = true;
  double pre_cte_ = 0;
  double sum_cte_ = 0;
  bool tuning_mode_ = false;
  
};

#endif /* PID_H */
