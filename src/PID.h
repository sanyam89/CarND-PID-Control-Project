#ifndef PID_H
#define PID_H
#include <vector>
using std::vector;


class PID {
 public:
  /**
   * Constructor
   */
    /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  vector<double> P, DP;
  int i;
  double total_error;
  double best_error;
  int update_count;
  int twiddle_count;
  int counter_thld;
  bool increment;
  
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  double Result();
  void Twiddle();

 private:

};

#endif  // PID_H