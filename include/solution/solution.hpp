#include <cmath>
#include <iostream>
#include <ostream>

constexpr float THRESHOLD = 0.004;

constexpr float PI = 3.14159265358979323846;

struct Angles
{
  float z; // Z axis rotation
  float y; // Y axis rotation
};

struct Motors
{
  int8_t z; // Z axis speed
  int8_t y; // Y axis speed
};

Angles point_to_angle(const Point& p)
{
  Angles angles;
  angles.z = std::atan2(p.y, p.x);
  angles.y = std::atan2(p.z, std::sqrt(p.x * p.x + p.y * p.y)); // x*x kinda faster than pow(x, 2)
  angles.z = (angles.z >= 0) ? angles.z : 2 * PI + angles.z;
  return angles;
}

// Expected motors to be way faster xDDD
class MotorDriver
{
private:
  double Kp, Kd;
  double prevErr;

public:
  MotorDriver(float Kp, float Kd) :
    Kp(Kp), Kd(Kd),
    prevErr(0.0)
  {
  }

  int8_t compute(float& targetAngle, float& currAngle, double& dt)
  {
    float err = targetAngle - currAngle;
    if (std::abs(err) < THRESHOLD) return 0;
    prevErr = err;

    if (std::abs(err) > PI) err = -err;

    float derivative = (err - prevErr) / dt;
    const int output = (Kp * err + Kd * derivative);

    if (output > 127 || err > 0.012) return 127;
    if (output < -128 || (err < 0 && std::abs(err) > 0.012)) return -128;
    return static_cast<int8_t>(output);
  }
};
