#include "tester.hpp"
#include "solution.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

std::atomic<Angles> targetAngles;
std::atomic<Angles> currPosition;

// requires tuning -_-
PD zAxisPD(680.0, 0.1);
PD yAxisPD(680.0,  0.1);

int solver(std::shared_ptr<backend_interface::Tester> tester, bool preempt)
{
  /* Short example you can remove it
  std::cout << (preempt ? "Preempt" : "Queue") << '\n';
  auto motor1 = tester->get_motor_1();
  auto motor2 = tester->get_motor_2();
  auto commands = tester->get_commands();
  motor1->add_data_callback([](const uint16_t& data) {
    std::cout << "Motor 1 data: " << static_cast<int>(data) << "\n";
  });
  motor2->add_data_callback([](const uint16_t& data) {
    std::cout << "Motor 2 data: " << static_cast<int>(data) << "\n";
  });
  commands->add_data_callback([](const Point& point) {
    std::cout << "Command point: (" << point.x << ", " << point.y << ", " << point.z << ")\n";
  });
  motor1->send_data(100);
  motor2->send_data(-50);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  */

  auto motor1 = tester->get_motor_1();
  auto motor2 = tester->get_motor_2();
  auto commands = tester->get_commands();
  auto prevTime = std::chrono::steady_clock::now();

  motor1->add_data_callback([](const uint16_t& encoder_value) {
    currPosition.store(Angles{(float)encoder_value*2*PI/4096, currPosition.load().y});
   });
  motor2->add_data_callback([](const uint16_t& encoder_value) {
    currPosition.store(Angles{currPosition.load().z, encoder_value*2*PI/4096});
  });

  targetAngles.store(point_to_angle(Point{1, 1, 1}));

  for (;;)
  {
    Motors mtr;
    Angles position = currPosition.load();
    Angles target = targetAngles.load();

    // more precise timing for PID
    auto currTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = currTime - prevTime;
    prevTime = currTime;
    double dt = elapsed.count();

    mtr.z = zAxisPD.compute(target.z, position.z, dt);
    mtr.y = yAxisPD.compute(target.y,position.y, dt);

    motor1->send_data(mtr.z);
    motor2->send_data(mtr.y);

    std::cout << "---------- TEST ----------" << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "Motors positions\n z:" << currPosition.load().z << " y: " << currPosition.load().y << std::endl;
    std::cout << "Target positions\n z:" << targetAngles.load().z << " y: " << targetAngles.load().y << std::endl;
    std::cout << "Motors speeds\n z: " << static_cast<int>(mtr.z) << " y: " << static_cast<int>(mtr.y)  << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
