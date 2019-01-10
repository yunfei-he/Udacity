#include "PID.h"

#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tuning) {
    twiddle_.Init(Kp, Ki, Kd);
    tuning_mode_ = tuning;
}

double PID::UpdateError(double cte, bool& reset) {
    sum_cte_ += cte;
    std::vector<double> coefficients = twiddle_.GetCoefficient();
    double pid = coefficients[0] * cte + coefficients[1] * sum_cte_;
    if (first_update_) { 
        first_update_ = false;
    } else {
        pid += coefficients[2] * (cte - pre_cte_);
    }
    pre_cte_ = cte;

    if (tuning_mode_) {
        twiddle_.UpdateError(cte);
        if (twiddle_.StateChanged()) {
            Reset();
            reset = true;
        } else {
            reset = false;
        }
    }

    // std::cout << "pid:" << pid << std::endl;
    while (pid > 1.0) pid -= 1.0;
    while (pid < -1.0) pid += 1.0;
    return pid;
}


void PID::Reset() {
   first_update_ = true;
   pre_cte_ = 0;
   sum_cte_ = 0;
}



