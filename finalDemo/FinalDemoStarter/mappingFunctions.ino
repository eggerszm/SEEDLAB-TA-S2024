double xValue(double heading, double distance, double currX) {
  return (distance * cos(heading)) + currX;
}

double yValue(double heading, double distance, double currY) {
  return (distance * sin(heading)) + currY;
}

double DesiredAngleXY(double currX, double currY, double desiredX, double desiredY) {
  return atan((desiredX - currX) / (desiredY - currY));
}

double DesiredDistanceXY(double currX, double currY, double desiredX, double desiredY) {
  return sqrt(sq(desiredX - currX) + sq(desiredY - currY));
}