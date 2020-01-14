#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <pigpiod_if2.h>

class HBridge {
 private: // pigpio
  int pi_;

  // Motor driver INPUT
  int pin_in1_;
  int pin_in2_;
  int pin_pwm_;

  // Motor Driver Type
  // 0: TB6612: 2 digital, 1 pwm
  // 1: TA7219P: 2 pwm
  int type_;


 public:

  HBridge(){
  }

  // Set Pin Assign for TB6612
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

  // Set Pin Assign for TA7219P
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
   * drive a motor
   * @param power power of a motor (-255 -- 255)
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
      // TA7219P
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