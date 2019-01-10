#ifndef TWIDDLE_H_
#define TWIDDLE_H_

#include <vector>

class Twiddle {

public:
  Twiddle();
  void Init(double Kp, double Ki, double Kd);
  std::vector<double> GetCoefficient() { return coefficient_;}
  bool StateChanged() {return state_changed_;}
  void UpdateError(double cte);

private:
  enum Operation {
     ADD = 0,
     MINUS = 1,
     LAST
  };
  int index_ = -1; //coefficient index to optimize
  double total_error_;
  double best_error_ = 0;
  bool first_iteration_ = true;
  int num_loops_ = 0;
  Operation op_ = LAST;
  bool state_changed_ = false;

  std::vector<double> coefficient_;
  std::vector<double> dp_;

};

#endif
