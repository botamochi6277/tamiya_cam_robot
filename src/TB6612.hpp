#ifndef TB6612_H
#define TB6612_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pigpiod_if2.h>

class TB6612 {
 private:
  int pi_;
  int pin_ain1_;
  int pin_ain2_;
  int pin_apwm_;
  int pin_bin1_;
  int pin_bin2_;
  int pin_bpwm_;
 public:
  static const bool A = 0;
  static const bool B = 1;

  TB6612(int pi,
         int pin_ain1,
         int pin_ain2,
         int pin_apwm,
         int pin_bin1,
         int pin_bin2,
         int pin_bpwm
        ) {
  pi_                 = pi;
  pin_ain1_           = pin_ain1;
  pin_ain2_           = pin_ain2;
  pin_apwm_           = pin_apwm;
  pin_bin1_           = pin_bin1;
  pin_bin2_           = pin_bin2;
  pin_bpwm_           = pin_bpwm;

    set_mode(pi_, pin_ain1_, PI_OUTPUT);
    set_mode(pi_, pin_ain2_, PI_OUTPUT);
    set_mode(pi_, pin_apwm_, PI_OUTPUT);
    set_mode(pi_, pin_bin1_, PI_OUTPUT);
    set_mode(pi_, pin_bin2_, PI_OUTPUT);
    set_mode(pi_, pin_bpwm_, PI_OUTPUT);
  }

  void drive(int motor, int power) {
    switch (motor) {
    case A :
      // Motor-A
      if (power > 0) {
        gpio_write(pi_, pin_ain1_, 1);
        gpio_write(pi_, pin_ain2_, 0);
      } else {
        gpio_write(pi_, pin_ain1_, 0);
        gpio_write(pi_, pin_ain2_, 1);
        power = -power;
      }
      set_PWM_dutycycle(pi_, pin_apwm_, power);
      break;

    case B:
      // Motor-B
      if (power > 0) {
        gpio_write(pi_, pin_bin1_, 1);
        gpio_write(pi_, pin_bin2_, 0);
      } else {
        gpio_write(pi_, pin_bin1_, 0);
        gpio_write(pi_, pin_bin2_, 1);
        power = -power;
      }
      set_PWM_dutycycle(pi_, pin_bpwm_, power);

      break;
    }
  }

  void serialCallback(const std_msgs::String::ConstPtr& msg){
  static int pwm0,pwm1;
  static char rot0,rot1;
  std::sscanf(msg->data.c_str(),"M0%c%03dM1%c%03d",&rot0,&pwm0,&rot1,&pwm1);

  ROS_INFO("I heard: [M0%c%03d, M1%c%03d]",rot0,pwm0,rot1,pwm1);

  if (rot0 == 'R'){
    pwm0 *=-1;
  }

  if (rot1 == 'R'){
    pwm1 *=-1;
  }
  drive(TB6612::A, pwm0);
  drive(TB6612::B, pwm1);

}

};
#endif