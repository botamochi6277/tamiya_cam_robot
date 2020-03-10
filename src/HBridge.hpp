/**
 * @file Hbridge.hpp

 * @brief Hbridge class controlling a DC-motor for ROS
**/

#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <pigpiod_if2.h>

/**
 * @brief Hbridge class controlling a DC-motor for ROS
 */
class HBridge {
 private: 
  int pi_; // pigpio id

  // INPUT pin numbers for motor driver
  int pin_in1_;
  int pin_in2_;
  int pin_pwm_;

  // Motor Driver Type
  // 0: TB6612 type needs two digital, one pwm signals.
  // 1: L298N type needs two pwm signals.
  int type_;

 public:

  /**
   * @brief default constructor (dummy)
   */
  HBridge(){
  }

  /**
   * @brief set gpio pin numbers for TB6612 type driver
   * @param pi      pigpio id
   * @param pin_in1 pin number connected to IN1 of TB6612
   * @param pin_in2 pin number connected to IN2 of TB6612
   * @param pin_pwm pin number connected to PWM of TB6612
   * @note You should set STBY pin HIGH.
   */
  void setPin(int pi,
          int pin_in1,
          int pin_in2,
          int pin_pwm) {
    pi_                 = pi;
    pin_in1_           = pin_in1;
    pin_in2_           = pin_in2;
    pin_pwm_           = pin_pwm;

    type_ = 0;

    set_mode(pi_, pin_in1_, PI_OUTPUT);
    set_mode(pi_, pin_in2_, PI_OUTPUT);
    set_mode(pi_, pin_pwm_, PI_OUTPUT);
  }

  /**
   * @brief set gpio pin numbers for L298N type driver
   * @param pi       pigpio id
   * @param pin_in1 pin number connected to IN1 of L298N
   * @param pin_in2 pin number connected to IN2 of L298N
   */
  void setPin(int pi,
          int pin_in1,
          int pin_in2) {
    pi_                 = pi;
    pin_in1_           = pin_in1;
    pin_in2_           = pin_in2;

    type_ = 1;

    set_mode(pi_, pin_in1_, PI_OUTPUT);
    set_mode(pi_, pin_in2_, PI_OUTPUT);
  }

  /**
   * @brief drive a motor
   * @param power power (duty ratio) of a motor (-255 -- 255)
   */
  void drive(int power) {
    switch (type_) {
    case 0 :
      // TB6612
      if (power > 0) {
        gpio_write(pi_, pin_in1_, 1);
        gpio_write(pi_, pin_in2_, 0);
      } else {
        gpio_write(pi_, pin_in1_, 0);
        gpio_write(pi_, pin_in2_, 1);
        power = -power;
      }
      set_PWM_dutycycle(pi_, pin_pwm_, power);
      break;

    case 1:
      // L298N
      if (power > 0) {
        set_PWM_dutycycle(pi_, pin_in1_, power);
        gpio_write(pi_, pin_in2_, 0);
      } else {
        power = -power;
        gpio_write(pi_, pin_in1_, 0);
        set_PWM_dutycycle(pi_, pin_in2_, power);
      }

      break;
    }
  }

};



#endif