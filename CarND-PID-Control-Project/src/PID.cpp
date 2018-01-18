#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) 
{
    mKp = Kp;
    mKi = Ki; 
    mKd = Kd;
    mbFirst = true; 
}

void PID::UpdateError(double cte) 
{
    if(mbFirst)
    {
	mICTE = cte;
	mDCTE = 0;
	mSquareErr = cte * cte; 
	mCTE = cte; 
	mbFirst = false; 
    }else
    {
	mICTE += cte; 
	mDCTE = cte - mCTE;
	mSquareErr += cte * cte; 
    }
    mCTE = cte; 
}

double PID::computeSteerValue()
{
    double alpha = -mKp * mCTE - mKi * mICTE - mKd * mDCTE;
    return alpha; 
}

double PID::TotalError() 
{
    return mSquareErr;    
}

