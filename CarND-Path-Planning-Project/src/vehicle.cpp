/*
    Apr. 8 2018, He Zhang, hxzhang1@ualr.edu

    A simple path planning class to guide vehicle in highway 
*/


#include "vehicle.h"
#include "global.h"
#include <cmath>

CVehicle::CVehicle(){}
CVehicle::CVehicle(int id, double x, double y, double s, double d, double v):
mid(id), mx(x), my(y), ms(s), md(d), mvs(v) // mvx(vx), mvy(vy)
{
    // mvs = sqrt(vx*vx + vy*vy); 
    mlane = (int)(d/4.); 
}

CVehicle::~CVehicle(){}

void CVehicle::move_at_constant_speed(double t)
{
    ms += mvs * t; 
    vector<double> tmpxy = getXY(ms,md); 
    mx = tmpxy[0]; 
    my = tmpxy[1]; 
}

void CVehicle::estimate(vector<CVehicle>& predictions, double& ref_vel, int& lane)
{
    // 1. first, whether a car ahead is so close 
    if(!find_close_vehicle_ahead(predictions))
    {	
	//
	// cout <<"vehicle.cpp: no close car ahead! Keep Speed"; 
	if(ref_vel < g_target_speed)
	    ref_vel += g_safe_acceleration; 
    }else{
	// 2. if there is a car ahead, consider to change lane or slow down    
	// TODO: check car begind that is close 
	if(prepare_move_left(predictions, lane, ref_vel))
	{
	    lane = lane - 1; 
	    cout <<"vehicle.cpp: move left to lane"<<lane<<", let's do it!"<<endl; 
	}else if(prepare_move_right(predictions, lane, ref_vel))
	{
	    lane = lane + 1; 
	    cout <<"vehicle.cpp: move right to lane"<<lane<<", let's do it!"<<endl; 
	}else
	{
	    cout <<"vehicle.cpp: move left & right not work, slow down!"<<endl; 
	    CVehicle vehicle_ahead; 
	    find_vehicle_ahead(predictions, lane, vehicle_ahead); 
	    if(ref_vel > vehicle_ahead.mvs) {
		ref_vel -= g_safe_acceleration; 
	    }else{
		cout <<"vehicle.cpp: too close! continue slow down!"<<endl; 
		if(vehicle_ahead.ms - ms < g_safe_distance/2) 
		    ref_vel -= g_safe_acceleration; 
	    }
	}
    }
    return ; 
}

bool CVehicle::is_safe_move(vector<CVehicle>& predictions, int lane, double& ref_vel)
{
    // check vehicle ahead 
    CVehicle vehicle_ahead; 
    if(find_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
	if(vehicle_ahead.ms - ms < g_safe_distance) 
	{
	    cout <<"vehicle: cannot move, ahead vehicle ds = "<<vehicle_ahead.ms - ms<<" too close!"<<endl;;
	    return false; 
	}else{
	    if(vehicle_ahead.mvs > ref_vel) 
		ref_vel += g_safe_acceleration; 
	}
    }

    // check vehicle behind 
    CVehicle vehicle_behind; 
    if(find_vehicle_behind(predictions, lane, vehicle_behind))
    {
	if(ms - vehicle_behind.ms < g_safe_distance)
	{
	    if(ms-vehicle_behind.ms < g_safe_distance / 2)
	    {
		cout <<"vehicle: cannot move, behind vehicle ds =  "<<ms - vehicle_behind.ms <<" too close!"<<endl; 
		return false; 
	    }else{
		if(ref_vel + g_safe_acceleration >= vehicle_behind.ms )
		{	
		    ref_vel += g_safe_acceleration; 
		}else{
		    cout <<"vehicle: cannot move, speed of behind vehicle is too fast"<<endl; 
		    return false; 
		}
	    }
	}
	if(ref_vel < vehicle_behind.mvs)
	    ref_vel += g_safe_acceleration; 
    }
    if(ref_vel >= g_target_speed)
	ref_vel -= g_safe_acceleration; 
    return true; 
}


bool CVehicle::prepare_move_right(vector<CVehicle>& predictions, int lane, double& ref_vel)
{
    if(lane == g_lane_right) return false; // cannot move to left 
    int new_lane = lane + 1; 
    double pre_vel = ref_vel; 
    if(is_safe_move(predictions, new_lane, ref_vel))
	return true; 
    ref_vel = pre_vel; 
    return false; 
}

bool CVehicle::prepare_move_left(vector<CVehicle>& predictions, int lane, double& ref_vel)
{
    if(lane == g_lane_left) return false; // cannot move to left 
    int new_lane = lane - 1; 
    double pre_vel = ref_vel; 
    if(is_safe_move(predictions, new_lane, ref_vel))
	return true; 
    ref_vel = pre_vel; 
    return false; 
}

bool CVehicle::find_vehicle_ahead(vector<CVehicle>& predictions, int lane, CVehicle& vehicle_ahead)
{
    bool ret = false; 
    vehicle_ahead.ms = ms + 100000; 
    for(int i=0; i<predictions.size(); i++)
    {
	CVehicle& tmp_car = predictions[i]; 
	if(tmp_car.mlane == lane && tmp_car.ms >= ms && tmp_car.ms < vehicle_ahead.ms)
	{
	    vehicle_ahead = tmp_car; 
	    ret = true; 
	}
    }
    return ret; 
}

bool CVehicle::find_close_vehicle_ahead(vector<CVehicle>& predictions)
{
     CVehicle vehicle_ahead;
     bool find_ahead = find_vehicle_ahead(predictions, this->mlane, vehicle_ahead); 
     if(find_ahead)
     {
	if(vehicle_ahead.ms - this->ms < g_safe_distance)
	{
	    return true; 
	}
	return false; 
     }
    return false; 
}

bool CVehicle::find_vehicle_behind(vector<CVehicle>& predictions, int lane, CVehicle& vehicle_behind)
{
    bool ret = false; 
    vehicle_behind.ms = ms - 100000; 
    for(int i=0; i<predictions.size(); i++)
    {
	CVehicle& tmp_car = predictions[i]; 
	if(tmp_car.mlane == lane && tmp_car.ms <= ms && tmp_car.ms > vehicle_behind.ms)
	{
	    vehicle_behind = tmp_car; 
	    ret = true; 
	}
    }
    return ret; 
}

bool CVehicle::find_close_vehicle_behind(vector<CVehicle>& predictions)
{
     CVehicle vehicle_behind;
     bool find_behind = find_vehicle_behind(predictions, this->mlane, vehicle_behind); 
     if(find_behind)
     {
	if(this->ms - vehicle_behind.ms < g_safe_distance)
	{
	    return true; 
	}
	return false; 
     }
    return false; 
}




