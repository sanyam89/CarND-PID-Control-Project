#include "PID.h"
#include <vector>
#include <iostream>
using std::cout;
using std::endl;
using std::vector;


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  P = {Kp_, Ki_, Kd_};
  DP = {Kp_/10.0, Ki_/10.0, Kd_/10.0};
  i = 0; // picking between P and I and D
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  total_error = 0.0;
  best_error = 0.0;
  update_count = 0;
  twiddle_count = 0;
  counter_thld = 0;
  increment = true;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  total_error += (cte*cte);
  update_count++;
  cout << "K_P = "<< P[0] << "  | K_I = " << P[1] << "  | K_D = " << P[2] << endl;
  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_error / update_count;  // TODO: Add your total error calc here!
}


double PID::Result()
{
  // This calculates the new input for the vehicle based on the PID gains
  
  double result = -1*(P[0]*p_error + P[1]*i_error + P[2]*d_error);
  if (result > 1.0)
    result = 1.0;
  else if (result < -1.0)
    result = -1.0;
  return result;
}

void PID::Twiddle()
{
  double current_error = TotalError(); // Get current error
  total_error = 0.0;		// Initialize the total error
  update_count  = 0;		// Initialize the update count for the next iteration
  twiddle_count++;
  
  if (best_error > 999)
  {
    best_error = current_error;
    P[i] += DP[i];
    return;
  }
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  if (current_error < best_error)
  {
    best_error = current_error;
    DP[i] *= 1.1;
    i = (i+1) % 3;		// Switching between P and I and D after updatng the DeltaP of the current coefficient
    increment = true;	// flag to be used to decrement the coefficient/gain if the new error is worse 
    
  }
  else		// New error is worse than the best error
  {
    if (increment)
      increment = false;      
    else
    {
      DP[i] *= 0.9;
      i = (i+1) % 3;	// Switching between P and I and D after updatng the DeltaP of the current coefficient
      increment = true;      
    }
  }
  if (increment)  P[i] += DP[i];
  else  P[i] -= 2*DP[i];	// Try decreasing the gain for the next iteration
}