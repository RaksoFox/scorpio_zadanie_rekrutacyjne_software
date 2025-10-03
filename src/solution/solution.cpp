#include "tester.hpp"
#include "solution.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

std::atomic<Angles> targetAngles;
std::atomic<Angles> currPosition;

MotorDriver zAxisDriver(1680.0, 0.2);
MotorDriver yAxisDriver(1480.0, 0.2);

std::vector<Angles> anglesQueue;

int solver(const std::shared_ptr<backend_interface::Tester>& tester, bool preempt) {
  auto motor1 = tester->get_motor_1();
  auto motor2 = tester->get_motor_2();
  auto commands = tester->get_commands();
  auto prevTime = std::chrono::steady_clock::now();

  motor1->add_data_callback([](const uint16_t& encoder_value) {
    currPosition.store(Angles{static_cast<float>(encoder_value) * 2 * PI / 4095, currPosition.load().y});
  });
  motor2->add_data_callback([](const uint16_t& encoder_value) {
    currPosition.store(Angles{currPosition.load().z, static_cast<float>(encoder_value) * 2 * PI / 4095});
  });

  commands->add_data_callback([&preempt](const Point& point) {
    std::cout << "Command point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    if (!preempt) {
      anglesQueue.push_back(point_to_angle(point));
    } else targetAngles.store(point_to_angle(point));
  });

  for (;;) {
    Motors mtr;
    Angles position = currPosition.load();
    Angles target = targetAngles.load();

    auto currTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = currTime - prevTime;
    prevTime = currTime;
    double dt = elapsed.count();

    mtr.z = -zAxisDriver.compute(target.z, position.z, dt);
    mtr.y = yAxisDriver.compute(target.y, position.y, dt);

    motor1->send_data(-mtr.z);
    motor2->send_data(mtr.y);

    if (!preempt && mtr.z == 0 && mtr.y == 0) {
      if (!anglesQueue.empty()) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(STOP_DELAY));
        targetAngles.store(anglesQueue.front());
        anglesQueue.erase(anglesQueue.begin());
      }
    }

    // Debug
    std::cout << "---------- DEBUG ----------" << std::endl;
    std::cout << "Delta: " << dt << std::endl;
    std::cout << "Motors positions\n z: " << currPosition.load().z << " y: " << currPosition.load().y << std::endl;
    std::cout << "Target position\n z: " << targetAngles.load().z << " y: " << targetAngles.load().y << std::endl;
    std::cout << "Motors speeds\n z: " << static_cast<int>(mtr.z) << " y: " << static_cast<int>(mtr.y) << std::endl;
    //

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
