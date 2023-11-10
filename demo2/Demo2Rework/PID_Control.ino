/*
PID Control
SEED Lab Group 9
Zander Eggers, Gideon Kukoyi, James Clark, Elijah Price
November 7th 2023

Uses basic control efforts to control any value using PID control
Set kP, kI, or kD to 0 if that control effort is unused
*/

double PID(double targetVal, double currVal, double desiredTsMs, double kP, double kI, double kD, double* integralVal, double* prevError) {
  double currError = targetVal - currVal;

  *integralVal += (desiredTsMs * currError) / 1000.0;

  double derivativeVal = (currError - *prevError) / desiredTsMs;
  *prevError = currError;

  return( (kP * currError) + (kI * *integralVal) + (kP * derivativeVal) );
}