/**
 * @file gpio_reading_taker.cpp
 *
 * @brief ROS Listener to read inputed digital signals.
 *
 * Under Development
**/
#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <iostream>
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
  ros::init(argc, argv, "gpio_reading_talker");
  ros::NodeHandle nh;
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_read = -1;
  if (!node_private.getParam("pin", pin_read)) {
    pin_read = 23; // Not found, set a default value
  }

  bool is_pull_up = false;
  node_private.getParam("pull_up", is_pull_up);

  float sampling_rate = 1.0f; // sampling ratio [Hz]
  node_private.getParam("rate", sampling_rate);

  ROS_INFO("GPIO Read: pin %02d", pin_read);

  // pigpio set pi mode
  set_mode(pi, pin, PI_INPUT);

  ros::Publisher pub = nh.advertise<std_msgs::Bool>((boost::format("tamiya1/gpio_r%02d") % pin_write).str(), 100);
  ros::Rate loop_rate(sampling_rate);

  while (ros::ok()) {
    std_msgs::Bool msg;
    msg.data = gpio_read(pi, pin);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  pigpio_stop(pi);
  return 0;
}