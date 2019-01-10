#ifndef CONST_H_
#define CONST_H_

const double LANE_WIDTH = 4.0;  // 4 meters 
const int POINT_TO_PREDICT = 50; // 50 points to generate per each circle
const double TIME_PER_CIRCLE = 0.02; //0.02 seconds
const double SPEED_LIMIT = 22.3; //m/s, around 50 MPH
const double SPEED_CHANGE_PER_CIRCLE = 0.081; //0.1001/0.02 leads to 5 m/s^2
const double MAX_ACC = 10.0; //10 m/s^2
const double MAX_JERK = 10.0; //10 m^s3
const double SAFETY_GAP_STRAIGHT_VEHICLE_AHEAD = 30; //meters
const double SAFETY_GAP_CHANGELANE_VEHICLE_AHEAD = 30; //meters
const double SAFETY_GAP_VEHICLE_BEHIND = 10; //meters
#endif 