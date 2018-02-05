/*
    Feb. 5 2018, He Zhang, hxzhang1@ualr.edu 

    twiddle to tune the PID 's parameters 
*/

#pragma once 

#include <vector>
#include "PID.h"

using namespace std; 

class CTwiddle
{
public:
    CTwiddle(PID* pid); 
    virtual ~CTwiddle(); 
    virtual bool addSample(double cte, double speed); 
    double computeSteerValue(); 
    bool stopIt(); 
    void setDP(vector<double>& dp); 
    double m_ref_speed; // expected speed 
    vector<double> mv_cur_p; // current parameters
    vector<double> mv_thre_dp; // threshold for dp 
    vector<double> mv_dp;  // dp 
    bool mb_reverse_search; // search dp inother direction 
    int m_update_index; 
    double mBestErr; 
    double mCurErr; 
    int mCnt; // count for number of data samples
    int mN; // data samples 
    PID* mpPID; 
};
