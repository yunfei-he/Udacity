#include "Twiddle.h"

#include <cmath>
#include <numeric>
#include <iostream>


const int LOOPS = 2000;
const double TOLERANCE = 0.02;

Twiddle::Twiddle() {
    coefficient_ = std::vector<double>(3, 0.0);
    dp_ = std::vector<double>(3, 0.0);
    dp_[0] = 0.1;
}

void Twiddle::Init(double Kp, double Ki, double Kd) {
    coefficient_[0] = Kp;
    coefficient_[1] = Ki;
    coefficient_[2] = Kd;
}

void Twiddle::UpdateError(double cte) {
    num_loops_ += 1;
    state_changed_ = false;

    if (std::abs(cte) < 0.001) {
        coefficient_[1] = 0.0;
    }
    if (std::abs(cte) > 3.0) {
        coefficient_[1] = 0.0;
    }
    //std::cout << "cte:" << cte << std::endl;
    if (std::accumulate(dp_.begin(), dp_.end(), 0.0) <= TOLERANCE) {
        std::cout << "You got optimal coefficient:" << coefficient_[0] << "," << coefficient_[1] << "," << coefficient_[2] << std::endl;
        return;
    }

    if (num_loops_ <= LOOPS / 2) {
        total_error_ += cte * cte;
    }

    if (num_loops_ == LOOPS) {
        std::cout << "error:" << best_error_ << "," << total_error_ << std::endl;
        if (first_iteration_) {
            std::cout << "first update" <<std::endl;
            best_error_ = total_error_;
            first_iteration_ = false;
            index_ = (index_ + 1) % coefficient_.size();
            coefficient_[index_] += dp_[index_];
            op_ = ADD;
        } else {
            if (best_error_ > total_error_) {
                std::cout << "got better error" <<std::endl;
                best_error_ = total_error_;
                dp_[index_] *= 1.1;

                index_ = (index_ + 1) % coefficient_.size();
                coefficient_[index_] += dp_[index_];
                op_ = ADD;
            } else {
                if (op_ == ADD) {
                    std::cout << "add" <<std::endl;
                    coefficient_[index_] -= 2 * dp_[index_];
                    op_ = MINUS;
                } else {
                    std::cout << "minus" <<std::endl;
                    coefficient_[index_] += dp_[index_];
                    dp_[index_] *= 0.9;

                    index_ = (index_ + 1) % coefficient_.size();
                    coefficient_[index_] += dp_[index_];
                    op_ = ADD;
                }
            } 
        }
        std::cout << "dp:" << dp_[0] << "," << dp_[1] << "," << dp_[2] << std::endl;
        std::cout << "coefficient:" << coefficient_[0] << "," << coefficient_[1] << "," << coefficient_[2] << std::endl << std::endl;
        total_error_ = 0;
        state_changed_ = true;
    }

    num_loops_ %= LOOPS;

}