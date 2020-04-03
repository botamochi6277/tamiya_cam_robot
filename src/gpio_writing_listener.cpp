/**
 * @file gpio_writing_listener.cpp
 *
 * @brief ROS Listener to output digital signals.
**/
#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <string>
#include <boost/format.hpp>
#include <chrono>
#include <thread>
#include <TB6612.hpp>
#include <pigpiod_if2.h>


class DigitalWriter {
 private:
  int pi_;
  int pin_;

 public:

  DigitalWriter(int pi, int pin) {
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
  ros::init(argc, argv, "gpio_writing_listener");
  ros::NodeHandle nh;
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_write = -1;
  if (!node_private.getParam("pin", pin_write)) {
    pin_write = 26; // Not found, set a default value
  }

  ROS_INFO("Write: pin %02d", pin_write);

  DigitalWriter mydigital(pi, pin_write);

  ros::Subscriber sub = nh.subscribe((boost::format("tamiya1/gpio_w%02d") % pin_write).str(),
                                     1000,
                                     &DigitalWriter::callback,
                                     &mydigital);
  ros::spin();
  mydigital.sleep();
  pigpio_stop(pi);
  return 0;
}