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
  * Twiddle params
  */
  double Tp;
  double Ti;
  double Td;

  long iter_count;
  double total_square_error;
  double best_error;

  int i;
  int step;

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
  * Initialize Twiddle parameters.
  */
  void InitTwiddle(double Tp, double Ti, double Td);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Get Steering Angle.
  */
  double getSteeringAngle();

  long getIterationCount();

  double getAverageError();

  double getBestError();

  void setBestError(double error);

  void updateTwiddleParams(double error);

  void printValues();
};

#endif /* PID_H */
