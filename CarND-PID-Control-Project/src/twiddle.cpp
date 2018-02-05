#include "twiddle.h"
#include <iostream>
#include <cmath>

CTwiddle::CTwiddle(PID* pid) :
mpPID(pid),
m_update_index(0),
mN(650),
mCnt(0),
mCurErr(0),
mBestErr(-1),
mb_reverse_search(false),
m_ref_speed(10)
{
    mv_cur_p.resize(3); 
    mv_cur_p[0] = pid->mKp; 
    mv_cur_p[1] = pid->mKi;
    mv_cur_p[2] = pid->mKd; 
}
CTwiddle::~CTwiddle(){}

void CTwiddle::setDP(vector<double>& dp)
{
    mv_dp = dp;
    mv_thre_dp = dp;
    for(int i=0; i<dp.size(); i++)
	mv_thre_dp[i] = 0.05 * mv_dp[i]; 
}	

bool CTwiddle::stopIt()
{
    double s = 0;
    for(int i=0; i<mv_dp.size(); i++)
	if(mv_dp[i] > mv_thre_dp[i])
	    return false; 
    return true; 
}

bool CTwiddle::addSample(double cte, double speed)
{
    mCnt++; 
    double speederr = (speed-m_ref_speed)*(speed-m_ref_speed);
    if(speed > m_ref_speed)
	speederr = 0; 
    mCurErr += fabs(cte) ;
    mCurErr += speederr;
    mpPID->UpdateError(cte); 
    if(mCnt >= mN)
    {
	if(mBestErr < 0) // first time 
	{
	    mBestErr = mCurErr; 
	    mv_cur_p[m_update_index] += mv_dp[m_update_index]; 
	}else // try to update 
	{
	    if(mCurErr < mBestErr) // a better solution is found 
	    {
		mBestErr = mCurErr; 
		mv_dp[m_update_index] *= 1.1; 
		if(mb_reverse_search)
		{
		    cout <<"update at index = "<<m_update_index<<" dp[index] = "<<mv_dp[m_update_index]<<endl; 
		}else
		{
		    cout<<"reverse update at index = "<<m_update_index<<" dp[index] = "<<mv_dp[m_update_index]<<endl;
		}
		m_update_index = (++m_update_index)%3;
		mv_cur_p[m_update_index] += mv_dp[m_update_index]; 
		mb_reverse_search = false; 
	    }else
	    {
		cout <<"failed to update bestErr: "<<mBestErr<<" curErr: "<<mCurErr<<endl;
		if(!mb_reverse_search)
		{
		   cout<<" try reverse direction!"<<endl;

		    // try another direction 
		    mv_cur_p[m_update_index] -= 2*mv_dp[m_update_index];
		    mb_reverse_search = true; 
		}else
		{
		    // completion of reverse search 
		    // recover P
		    mv_cur_p[m_update_index] += mv_dp[m_update_index]; 
		    
		    // reduce the range at dp[m_update_index]
		    mv_dp[m_update_index] *= 0.8; 
		    cout <<"failed to update at index = "<<m_update_index<<" dp[index] = "<<mv_dp[m_update_index]<<endl;

		    m_update_index = (++m_update_index)%3;
		    mv_cur_p[m_update_index] += mv_dp[m_update_index]; 
		    mb_reverse_search = false; 
		}
	    }
	}

	// reset 
	mCurErr = 0;
	mCnt = 0; 
	mpPID->Init(mv_cur_p[0], mv_cur_p[1], mv_cur_p[2]); // reset PID controller 

	return true; 
    }else
    {
	
    }
    return false; 
}

double CTwiddle::computeSteerValue()
{
    return mpPID->computeSteerValue(); 
}

