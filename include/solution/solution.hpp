#include <cmath>
#include <iostream>
#include <ostream>
#include <algorithm>

constexpr float DEDZONE = 0.01;
constexpr float THRESHOLD = 0.012;
constexpr float ALPHA = 0.08;
// constexpr int STOP_DELAY = 1000;

constexpr float PI = 3.14159265358979323846;

struct Angles {
  float z; // Z axis rotation
  float y; // Y axis rotation
};

struct Motors {
  int8_t z; // Z axis speed
  int8_t y; // Y axis speed
};

Angles point_to_angle(const Point& p) {
  Angles angles;
  angles.z = std::atan2(p.y, p.x);
  angles.y = std::atan2(p.z, std::sqrt(p.x * p.x + p.y * p.y)); // x*x kinda faster than pow(x, 2)
  angles.z = (angles.z >= 0) ? angles.z : 2 * PI + angles.z;
  angles.y = (angles.y >= 0) ? angles.y : 2 * PI + angles.y;
  return angles;
}

// Expected motors to be way faster xDDD
class MotorDriver {
private:
  double Kp, Kd;
  double prevErr;
  double prevOut;

public:
  MotorDriver(float Kp, float Kd) :
    Kp(Kp), Kd(Kd),
    prevErr(0.0),
    prevOut(0.0) {}

  int8_t compute(const float& targetAngle, const float& currAngle, double& dt) {
    float err = targetAngle - currAngle;
    prevErr = err;

    // use optimal route
    if (std::abs(err) > PI) err = -err;

    float derivative = (err - prevErr) / dt;
    float outSpeed = (Kp * err + Kd * derivative);

    if (std::abs(err) < DEDZONE) outSpeed = 0;
    if (err > THRESHOLD) outSpeed = 130;
    if (err < 0 && std::abs(err) > THRESHOLD) outSpeed = -130;
    // smoothing I guess
    outSpeed = ALPHA * outSpeed + (1.0 - ALPHA) * prevOut;
    prevOut = outSpeed;
    return static_cast<int8_t>(std::clamp(static_cast<int>(outSpeed), -128, 127));
  }
};
