#ifndef VEHICLE_H
#define VEHICLE_H

#include <cmath>

#include "Helper.h"

class Vehicle {
public:
    Vehicle() {}
    Vehicle(int id, double x, double y, double s, double d):
           id_(id), x_(x), y_(y), s_(s), d_(d) {}
           
    void SetYawAndSpeed(double yaw, double speed) {
        yaw_ = Helper::deg2rad(yaw);
        speed_ = speed;
        x_speed_ = speed_ * cos(yaw);
        y_speed_ = speed_ * sin(yaw);
    }
    void SetXYSpeed(double x_speed, double y_speed) {
        x_speed_ = x_speed;
        y_speed_ = y_speed;
        speed_ = std::hypot(x_speed_, y_speed_);
        yaw_ = atan2(y_speed_, x_speed_);
    }

public:
    int id_;
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
    double x_speed_;
    double y_speed_;
};
#endif