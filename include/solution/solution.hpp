#include <cmath>
#include <iostream>
#include <ostream>
#include <algorithm>

constexpr float THRESHOLD = 0.008;
constexpr float ALPHA = 0.1;

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
  double prevOut;

public:
  MotorDriver(float Kp, float Kd) :
    Kp(Kp), Kd(Kd),
    prevErr(0.0),
    prevOut(0.0)
  {
  }

  int8_t compute(const float& targetAngle, const float& currAngle, double& dt)
  {
    float err = targetAngle - currAngle;
    if (std::abs(err) < THRESHOLD) return 0;
    prevErr = err;

    if (std::abs(err) > PI) err = -err;

    float derivative = (err - prevErr) / dt;
    float output = (Kp * err + Kd * derivative);

    if (err > 0.012) output = 127;
    if (err < 0 && std::abs(err) > 0.012) output = -128;

    // smoothing I guess
    output = ALPHA * output + (1.0 - ALPHA) * prevOut + 10;
    prevOut = output;
    return static_cast<int8_t>(std::clamp(static_cast<int>(output), -128, 127));
  }
};
