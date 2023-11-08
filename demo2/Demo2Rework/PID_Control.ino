double PID(double targetVal, double currVal, double desiredTsMs, double kP, double kI, double kD, double* integralVal, double* prevError) {
  double currError = targetVal - currVal;

  *integralVal += (desiredTsMs * currError) / 1000.0;

  double derivativeVal = (currError - *prevError) / desiredTsMs;
  *prevError = currError;

  return( (kP * currError) + (kI * *integralVal) + (kP * derivativeVal) );
}