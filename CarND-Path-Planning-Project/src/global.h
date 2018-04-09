

#pragma once
#include <vector>
#include <cmath>

using namespace std; 


// For converting back and forth between radians and degrees.
extern double deg2rad(double x); // { return x * pi() / 180; }
extern double rad2deg(double x); // { return x * 180 / pi(); }
extern double distance(double x1, double y1, double x2, double y2);

// find the cloest waypoint to the given point (x, y)
extern int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y); 

// find the next waypoint to the given pose (x, y, theta)
extern int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y); 

// Transform from Frenet s,d coordinates to Cartesian x,y
extern vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
extern vector<double> getXY(double s, double d);

// read from disk 
extern  vector<double> g_map_waypoints_x;
extern  vector<double> g_map_waypoints_y;
extern  vector<double> g_map_waypoints_s;
extern  vector<double> g_map_waypoints_dx;
extern  vector<double> g_map_waypoints_dy;

// global parameters
extern double g_lane_width; 
extern double g_safe_distance;
extern double g_safe_acceleration;
extern double g_target_speed;
extern int g_lane_left; 
extern int g_lane_right; 
