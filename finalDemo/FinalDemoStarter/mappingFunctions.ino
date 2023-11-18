double xValue(double heading, double distance) {
  return distance * cos(heading);
}

double yValue(double heading, double distance) {
  return distance * sin(heading);
}

double DesiredAngleXY(double currX, double currY, double desiredX, double desiredY) {
  return atan((desiredX - currX) / (desiredY - currY));
}

double DesiredDistanceXY(double currX, double currY, double desiredX, double desiredY) {
  return sqrt(sq(desiredX - currX) + sq(desiredY - currY));
}