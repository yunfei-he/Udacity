#ifndef HELPER_H_
#define HELPER_H_

#include <math.h>
#include <vector>

namespace Helper {
// Calculates Jerk Minimizing Trajectory for start, end and T.
std::vector<double> JMT(const std::vector<double>& start, const std::vector<double>& end, double T);

// Integratea coefficient with delta time t, and returns a pair of (s,d)
std::pair<double, double> Integrate(const std::vector<double>& traj, double t);

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

}

#endif