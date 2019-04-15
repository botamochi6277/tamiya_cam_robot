#include "ros/ros.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <pigpiod_if2.h>

class neopixel{

};


int main(int argc, char **argv) {
  ros::init(argc, argv, "test_run");
  ros::NodeHandle nh;

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    std::printf("cannot connect pigpiod\n");
    return -1;
  }




  
  pigpio_stop(pi);
  // ROS_INFO("Finish driving.");
  return 0;
}