#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
    
  double computeSteerValue();

  double mKp, mKi, mKd;
  double mICTE; // Integral
  double mDCTE; // Differential 
  double mCTE;  // Proportional 
  double mSquareErr; // accumulated square err
  int mCnt; // number of errors 
  bool mbFirst; 
};

#endif /* PID_H */
