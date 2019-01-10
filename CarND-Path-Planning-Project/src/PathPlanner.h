#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "Vehicle.h"

class PathPlanner {
private:
    Vehicle reference_car_;
    std::vector<Vehicle> traffic_;
    std::vector<double> pre_xs_;
    std::vector<double> pre_ys_;
    double pre_end_s_;
    double pre_end_y_;

    std::vector<double> waypoint_xs_;
    std::vector<double> waypoint_ys_;
    std::vector<double> waypoint_ss_;
    std::vector<double> waypoint_dxs_;
    std::vector<double> waypoint_dys_;

public:
    PathPlanner();
    void SetWayPoints(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& ss, 
                      const std::vector<double>& dxs, const std::vector<double>& dys);
    void GetPath(const std::vector<Vehicle>& traffic, const Vehicle& ego_car, 
                 const std::vector<double>& previous_xs, const std::vector<double>& previous_ys,
                 double previous_end_s, double previous_end_d,
                 std::vector<double> *xs, std::vector<double> *ys);

private:
   Vehicle FindNearestVehicleAhead(int lane);
   Vehicle FindNearestVehicleBehind(int lane);
   int GetLaneNumber(double d);
   bool CanGoStraight();
   bool CanChangeToLeftLane(double* s);
   bool CanChangeToRightLane(double* s);
   void CalcNewPoint(std::vector<double> *xs, std::vector<double> *ys);
};
#endif