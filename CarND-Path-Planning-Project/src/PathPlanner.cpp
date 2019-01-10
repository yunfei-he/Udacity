#include "PathPlanner.h"

#include <iostream>

#include "const.h"
#include "Helper.h"
#include "spline.h"

PathPlanner::PathPlanner() {
    reference_car_.speed_ = 0.0;
    reference_car_.d_ = 1.5 * LANE_WIDTH; //put car in the middle
}

void PathPlanner::SetWayPoints(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& ss, 
                      const std::vector<double>& dxs, const std::vector<double>& dys) {
    waypoint_xs_ = xs;
    waypoint_ys_ = ys;
    waypoint_ss_ = ss;
    waypoint_dxs_ = dxs;
    waypoint_dys_ = dys;
}

void PathPlanner::GetPath(const std::vector<Vehicle>& traffic, const Vehicle& ego_car, 
                 const std::vector<double>& previous_xs, const std::vector<double>& previous_ys,
                 double previous_end_s, double previous_end_d,
                 std::vector<double> *xs, std::vector<double> *ys) {
    traffic_ = traffic;
    pre_xs_ = previous_xs;
    pre_ys_ = previous_ys;

    if (previous_xs.empty()) {
        reference_car_ = ego_car;
        //reference_car_.speed_ = SPEED_LIMIT / 2;
    } else {
        reference_car_.s_ = previous_end_s;
        reference_car_.d_ = previous_end_d;
    }
   
   // take previous leftover
   for (int i = 0; i < previous_xs.size(); ++i) {
       xs->push_back(previous_xs[i]);
       ys->push_back(previous_ys[i]);
   }

   // calcualte new points
   CalcNewPoint(xs, ys);

}

void PathPlanner::CalcNewPoint(std::vector<double> *xs, std::vector<double> *ys) {
    // calculate new speed and new lane
    double new_speed = reference_car_.speed_;
    int new_lane = GetLaneNumber(reference_car_.d_);
    Vehicle v = FindNearestVehicleAhead(new_lane);
    double delta_s = v.s_ - reference_car_.s_;
    // if safe to go straight
    if (delta_s > SAFETY_GAP_STRAIGHT_VEHICLE_AHEAD) {
        new_speed = std::min(new_speed + SPEED_CHANGE_PER_CIRCLE, SPEED_LIMIT);
    } else {
        double left_s;
        bool can_change_to_left = CanChangeToLeftLane(&left_s);
        double right_s;
        bool can_change_to_right = CanChangeToRightLane(&right_s);
        if (can_change_to_left && can_change_to_right) {
            if (left_s > right_s) {
                new_lane -= 1;
            } else {
                new_lane += 1;
            }
        } else if (can_change_to_left) {
            new_lane -= 1;
        } else if (can_change_to_right) {
            new_lane += 1;
        } else if (delta_s < SAFETY_GAP_STRAIGHT_VEHICLE_AHEAD / 2) {
            new_speed -= SPEED_CHANGE_PER_CIRCLE;
        } else {
            if (new_speed >= v.speed_) {
                new_speed -= std::min((new_speed - v.speed_), SPEED_CHANGE_PER_CIRCLE);
            } 
        }
    }


    std::cout << std::endl << "new speed:" << new_speed << std::endl;
    reference_car_.speed_ = new_speed;

    // calculate spline line starting x, y and yaw
    double start_x = reference_car_.x_;
    double start_y = reference_car_.y_;
    double start_yaw = reference_car_.yaw_;
    std::vector<double> v_x;
    std::vector<double> v_y; 
    if (pre_xs_.size() < 2) {
        v_x.push_back(start_x - cos(start_yaw));
        v_x.push_back(start_x);

        v_y.push_back(start_y - sin(start_yaw));
        v_y.push_back(start_y);
    } else {
        size_t pre_size = pre_xs_.size();
        v_x.push_back(pre_xs_[pre_size-2]);
        v_x.push_back(pre_xs_[pre_size-1]);

        v_y.push_back(pre_ys_[pre_size-2]);
        v_y.push_back(pre_ys_[pre_size-1]);

        start_x = pre_xs_.back();
        start_y = pre_ys_.back();
        start_yaw = atan2(pre_ys_[pre_size-1] - pre_ys_[pre_size-2], pre_xs_[pre_size-1] - pre_xs_[pre_size-2]);
    }

    // add evenly 30 spaces points from starting point
    for (int i = 1; i <= 3; ++i) {
        std::vector<double> xy = Helper::getXY(reference_car_.s_ + 30.0 * i, LANE_WIDTH * (0.5 + new_lane), 
                                            waypoint_ss_, waypoint_xs_, waypoint_ys_);
        v_x.push_back(xy[0]);
        v_y.push_back(xy[1]);
    }

    // shift x,y to car local frame
    for (int i = 0; i < v_x.size(); ++i) {
        double shift_x = v_x[i] - start_x;
        double shift_y = v_y[i] - start_y;

        v_x[i] = shift_x * cos(-start_yaw) - shift_y * sin(-start_yaw);
        v_y[i] = shift_x * sin(-start_yaw) + shift_y * cos(-start_yaw);
    }
    
    tk::spline ts;
    ts.set_points(v_x, v_y);

    // add new points until it reachs 50
    double target_x = 30.0;
    double target_y = ts(target_x);
    double target_dist = std::hypot(target_x, target_y);
    double N = target_dist / (TIME_PER_CIRCLE * new_speed);
    double delta_x = target_x / N;

    for (int i = 1; i <= POINT_TO_PREDICT - pre_xs_.size(); ++i) {
        double x = delta_x * i;
        double y = ts(x);

        // transfer local frame to global frame
        double new_x = start_x + x * cos(start_yaw) - y * sin(start_yaw);
        double new_y = start_y + x * sin(start_yaw) + y * cos(start_yaw);
        std::cout << "new x, y: " << new_x << "," << new_y << std::endl;
        xs->push_back(new_x);
        ys->push_back(new_y);
    }

}

Vehicle PathPlanner::FindNearestVehicleAhead(int lane) {
    Vehicle ans;
    ans.s_ = std::numeric_limits<float>::max();
    ans.speed_ = 0;
    for (const auto& v : traffic_) {
        if (lane == GetLaneNumber(v.d_)) {
            double s = v.s_;
            s += v.speed_ * TIME_PER_CIRCLE * pre_xs_.size();
            if (s > reference_car_.s_ && s < ans.s_) {
                ans.s_ = s;
                ans.speed_ = v.speed_;
            }
        }
    }
    return ans;
}

Vehicle PathPlanner::FindNearestVehicleBehind(int lane) {
    Vehicle ans;
    ans.s_ = -100;
    ans.speed_ = 0;
    for (const auto& v : traffic_) {
        if (lane == GetLaneNumber(v.d_)) {
            double s = v.s_;
            s += v.speed_ * TIME_PER_CIRCLE * pre_xs_.size();
            if (s < reference_car_.s_ && s > ans.s_) {
                ans.s_ = s;
            }
        }
    }
    return ans;
}

int PathPlanner::GetLaneNumber(double d) {
    if (d <= LANE_WIDTH) {
        return 0;
    } else if (d >= 2 * LANE_WIDTH) {
        return 2;
    } else {
        return 1;
    }
}

bool PathPlanner::CanGoStraight() {
     int lane_num = GetLaneNumber(reference_car_.d_);
     Vehicle v = FindNearestVehicleAhead(lane_num);
     return (v.s_ - reference_car_.s_) >= SAFETY_GAP_STRAIGHT_VEHICLE_AHEAD;
}

bool PathPlanner::CanChangeToLeftLane(double *s) {
    int lane_num = GetLaneNumber(reference_car_.d_);
    if (lane_num == 0) {
        return false;
    }

    Vehicle v = FindNearestVehicleAhead(lane_num-1);
    if ((v.s_ - reference_car_.s_) < SAFETY_GAP_CHANGELANE_VEHICLE_AHEAD) {
        return false;
    }
    *s = v.s_;

    v = FindNearestVehicleBehind(lane_num-1);
    return reference_car_.s_ - v.s_ >= SAFETY_GAP_VEHICLE_BEHIND;
}
bool PathPlanner::CanChangeToRightLane(double *s) {
    int lane_num = GetLaneNumber(reference_car_.d_);
    if (lane_num == 2) {
        return false;
    }

    Vehicle v = FindNearestVehicleAhead(lane_num + 1);
    if ((v.s_ - reference_car_.s_) < SAFETY_GAP_CHANGELANE_VEHICLE_AHEAD) {
        return false;
    }
    *s = v.s_;

    v = FindNearestVehicleBehind(lane_num + 1);
    return reference_car_.s_ - v.s_ >= SAFETY_GAP_VEHICLE_BEHIND;
}

