/**
 * @brief Drive brushed DC-motors with TB6612
 * @author botamochi6277
 */

#include "ros/ros.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <TB6612.hpp>
#include <pigpiod_if2.h>

int main(int argc, char **argv) {

  // establish ros node
  ros::init(argc, argv, "test_run");
  ros::NodeHandle nh;

  // connect to pigpio
  int pi;
  pi = pigpio_start(NULL, NULL);

  if (pi < 0) {
    std::printf("cannot connect pigpiod\n");
    return -1;
  }

  // define pin assignment for TB6612
  int pin_ain1, pin_ain2, pin_apwm, pin_bin1, pin_bin2, pin_bpwm;

  // Read pin assignment from ROSPARAM
  std::vector<std::string> pinouts;
  if (nh.getParam("pigpio", pinouts)) {
    for (int i = 0; i < pinouts.size(); ++i) {
      if (pinouts.at(i) == "A_IN1") {
        pin_ain1 = i;
      }
      if (pinouts.at(i) == "A_IN2") {
        pin_ain2 = i;
      }
      if (pinouts.at(i) == "A_PWM") {
        pin_apwm = i;
      }
      if (pinouts.at(i) == "B_IN1") {
        pin_bin1 = i;
      }
      if (pinouts.at(i) == "B_IN2") {
        pin_bin2 = i;
      }
      if (pinouts.at(i) == "B_PWM") {
        pin_bpwm = i;
      }
    }
  } else {
    ROS_ERROR("No pigpio param");
  }

  // establish TB6612 class
  TB6612 driver(pi, pin_ain1, pin_ain2, pin_apwm, pin_bin1, pin_bin2, pin_bpwm);
  ROS_INFO("TB6612 is working, A_IN1: %02d, A_IN2: %02d, A_PWM: %02d, B_IN1: %02d, B_IN2: %02d, B_PWM: %02d",
           pin_ain1, pin_ain2, pin_apwm, pin_bin1, pin_bin2, pin_bpwm);

  // drive the motors
  ROS_INFO("Go forward.");
  driver.drive(TB6612::A, 128);// right motor
  driver.drive(TB6612::B, 128);// left motor
  std::this_thread::sleep_for(std::chrono::seconds(3));

  ROS_INFO("Go backward.");
  driver.drive(TB6612::A, -128);
  driver.drive(TB6612::B, -128);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  ROS_INFO("Turn right.");
  driver.drive(TB6612::A, -128);
  driver.drive(TB6612::B, 128);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  ROS_INFO("Turn left.");
  driver.drive(TB6612::A, 128);
  driver.drive(TB6612::B, -128);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Stop motors
  driver.drive(TB6612::A, 0);
  driver.drive(TB6612::B, 0);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // disconnect pigpio
  pigpio_stop(pi);
  ROS_INFO("Finish driving.");
  return 0;
}
// %EndTag(FULLTEXT)%