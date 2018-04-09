/*
    Apr. 6 2018, He Zhang, hxzhang1@ualr.edu

    A simple path planning class to guide vehicle in highway 
*/

#pragma once

#include <vector>
#include <string>
#include <iostream>

using namespace std; 

class CVehicle
{
public:
    CVehicle(); 
    // CVehicle(double x, double y, double s, double d, double vx, double vy, double lane); 
    CVehicle(int id, double x, double y, double s, double d, double v); 
    ~CVehicle(); 
   
    void estimate(vector<CVehicle>& predictions, double& ref_vel, int& lane); 

    bool find_close_vehicle_behind(vector<CVehicle>& predictions); 
    bool find_close_vehicle_ahead(vector<CVehicle>& predictions); 

    bool find_vehicle_ahead(vector<CVehicle>& predictions, int lane, CVehicle& vehicle_ahead); 
    bool find_vehicle_behind(vector<CVehicle>& predictions, int lane, CVehicle& vehicle_ahead); 

    void move_at_constant_speed(double time); 
    bool prepare_move_left(vector<CVehicle>& predictions, int lane, double& ref_vel); 
    bool prepare_move_right(vector<CVehicle>& predictions, int lane, double& ref_vel); 
    bool is_safe_move(vector<CVehicle>& predictions, int lane, double& ref_vel); 

    int mid; 
    int mlane; 
    double mvs;
    // double mvx;
    // double mvy;
    double mx;
    double my;
    double ms; 
    double md;
    double myaw; // heading 

};
