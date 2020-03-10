/**
 * @file wheels_listener.cpp

 * @brief ROS Listener to control wheels
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <HBridge.hpp>
#include <pigpiod_if2.h>

/**
 * @brief Wheels/Tracks Controller for Two DC motors with H-bridge drivers
 */
class Wheels {
 private:
  HBridge right_;
  HBridge left_;
 public:

  /**
   * @brief default constructor (dummy)
   */
  Wheels() {
  }

  /**
   * @brief set gpio pin numbers for TB6612 type driver
   * @param pi       pigpio id
   * @param pin_ain1 pin number connected to AIN1 of TB6612
   * @param pin_ain2 pin number connected to AIN2 of TB6612
   * @param pin_apwm pin number connected to PWMA of TB6612
   * @param pin_bin1 pin number connected to BIN1 of TB6612
   * @param pin_bin2 pin number connected to BIN2 of TB6612
   * @param pin_bpwm pin number connected to PWMB of TB6612
   *
   * @note You should set STBY pin HIGH.
   */
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

  /**
   * @brief set gpio pin numbers for L298N type driver
   * @param pi       pigpio id
   * @param pin_ain1 pin number connected to AIN1 (M1A) of L298N
   * @param pin_ain2 pin number connected to AIN2 (M2A) of L298N
   * @param pin_bin1 pin number connected to BIN1 (M1B) of L298N
   * @param pin_bin2 pin number connected to BIN2 (M2B) of L298N
   */
  void setPin(int pi,
              int pin_ain1,
              int pin_ain2,
              int pin_bin1,
              int pin_bin2) {
    right_.setPin(pi, pin_ain1, pin_ain2);
    left_.setPin(pi, pin_bin1, pin_bin2);
  }

  /**
   * @brief drive motors
   * @param power_r PWM duty ratio for right motor (-255 -- +255)
   * @param power_l PWM duty ratio for left motor (-255 -- +255)
   */
  void drive(int power_r, int power_l) {
    right_.drive(power_r);
    left_.drive(power_l);
  }

  /**
   * @brief callback of ros subscriber to drive motors
   * @param msg rosmassage(geometry_msgs::Twist) having acceleration and rotation efforts (0--1.0)
   */
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    static int pwm_r, pwm_l;
    // pwm_r = 255*(msg->linear.x +0.5*wheel_distance_*msg->angular.z);
    // pwm_l = 255*(msg->linear.x -0.5*wheel_distance_*msg->angular.z);
    pwm_r = 255 * (msg->linear.x + msg->angular.z);
    pwm_l = 255 * (msg->linear.x - msg->angular.z);

    // clip duty ratio
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
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_ain1, pin_ain2, pin_bin1, pin_bin2;
  int pin_apwm = -1;
  int pin_bpwm = -1;

  // read pin assignments
  if (!node_private.getParam("AIN1", pin_ain1)) {
    pin_ain1 = 17; // Not found, set a default value
  }
  if (!node_private.getParam("AIN2", pin_ain2)) {
    pin_ain2 = 18; // Not found, set a default value
  }
  if (!node_private.getParam("BIN1", pin_bin1)) {
    pin_bin1 = 27; // Not found, set a default value
  }
  if (!node_private.getParam("BIN2", pin_bin2)) {
    pin_bin2 = 22; // Not found, set a default value
  }

  // PWM pins
  if (node_private.getParam("PWMA", pin_apwm)) {
  }
  if (node_private.getParam("PWMB", pin_bpwm)) {
  }


  // Set pins
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
    // L298N
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