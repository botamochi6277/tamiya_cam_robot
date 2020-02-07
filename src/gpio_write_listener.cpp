#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <TB6612.hpp>
#include <pigpiod_if2.h>


class Digital {
 private:
  int pi_;
  int pin_;

 public:

  Digital(int pi, int pin) {
    pi_ = pi;
    pin_ = pin;
    set_mode(pi_, pin_, PI_OUTPUT);
  }

  void sleep() {
    set_mode(pi_, pin_, PI_INPUT);
  }

  void callback(const std_msgs::Bool::ConstPtr& msg) {
    gpio_write(pi_, pin_ , msg->data);
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gpio_write_listener");
  ros::NodeHandle nh;
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_write = 26;
  std::string pin_name = "buzzer";
  if (node_private.getParam("name", pin_name)) {
    //
  }

  // Read pin assign
  std::vector<std::string> pinouts;
  if (nh.getParam("pigpio", pinouts)) {
    for (int i = 0; i < pinouts.size(); ++i) {
      if (pinouts.at(i) == pin_name) {
        pin_write = i;
      }
    }
  } else {
    ROS_ERROR("No pigpio param");
  }

  ROS_INFO("%s - pin %02d", pin_name.c_str(), pin_write);

  Digital mydigital(pi, pin_write);

  ros::Subscriber sub = nh.subscribe("tamiya1/" + pin_name, 1000,
                                     &Digital::callback, &mydigital);
  ros::spin();
  mydigital.sleep();
  pigpio_stop(pi);
  return 0;
}