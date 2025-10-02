#include <cmath>

struct Angles {
  double x;  // X angle
  double y;  // Y angle
};

Angles point_to_angle(Point point)
{
  Angles angles;
  angles.x = atan2(point.x, -point.y);
  angles.y = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)); // x*x kinda faster than pow(x, 2)

  return angles;
}
