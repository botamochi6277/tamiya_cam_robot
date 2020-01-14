#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <HBridge.hpp>
#include <pigpiod_if2.h>

class Wheels {
 private:
  HBridge right_;
  HBridge left_;
 public:
  Wheels() {
  }

  // TB6612
  void setPin(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_apwm,
         int pin_bin1,
         int pin_bin2,
         int pin_bpwm) {
    right_.setPin(pi, pin_ain1, pin_ain2, pin_apwm);
    left_.setPin(pi, pin_bin1, pin_bin2, pin_bpwm) ;
  }

  // 2x TA7219P
  void setPin(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_bin1,
         int pin_bin2) {
    right_.setPin(pi, pin_ain1, pin_ain2);
    left_.setPin(pi, pin_bin1, pin_bin2);
  }

  void drive(int power_r, int power_l) {
    right_.drive(power_r);
    left_.drive(power_l);
  }

  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    static int pwm_r, pwm_l;
    // pwm_r = 255*(msg->linear.x +0.5*wheel_distance_*msg->angular.z);
    // pwm_l = 255*(msg->linear.x -0.5*wheel_distance_*msg->angular.z);
    pwm_r = 255 * (msg->linear.x + msg->angular.z);
    pwm_l = 255 * (msg->linear.x - msg->angular.z);

    // limit duty ratio
    if (std::abs(pwm_r) > 255) {
      float c = 255.0 / std::abs(pwm_r);
      pwm_r *= c;
      pwm_l *= c;
    }

    if (std::abs(pwm_l) > 255) {
      float c = 255.0 / std::abs(pwm_l);
      pwm_r *= c;
      pwm_l *= c;
    }
    drive(pwm_r, pwm_l);

    // ROS_INFO("motor: %d,%d",pwm_r,pwm_l);
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "wheels_listener");
  ros::NodeHandle nh;

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_ain1, pin_ain2, pin_bin1, pin_bin2;
  int pin_apwm = -1;
  int pin_bpwm = -1;

  // Read pin assign
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
    return -1;
  }

  Wheels wheels;
  ROS_INFO("DC-MOTOR DRIVER is working");
  if (pin_apwm >= 0) {
    // TB6612
    wheels.setPin(pi,
                  pin_ain1, pin_ain2, pin_apwm,
                  pin_bin1, pin_bin2, pin_bpwm);
    ROS_INFO("R_IN1: %02d, R_IN2: %02d, R_PWM: %02d,L_IN1: %02d, L_IN2: %02d, L_PWM: %02d",
             pin_ain1, pin_ain2, pin_apwm, pin_bin1, pin_bin2, pin_bpwm);
  } else {
    // 2x TA7219P
    wheels.setPin(pi,
                  pin_ain1, pin_ain2,
                  pin_bin1, pin_bin2);
    ROS_INFO("R_IN1: %02d, R_IN2: %02d, L_IN1: %02d, L_IN2: %02d",
             pin_ain1, pin_ain2, pin_bin1, pin_bin2);
  }


  ros::Subscriber sub = nh.subscribe("tamiya1/cmd_vel", 1000,
                                     &Wheels::twistCallback, &wheels);

  ros::spin();
  ROS_INFO("Finish driving.");

  wheels.drive(0, 0);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  pigpio_stop(pi);

  return 0;
}