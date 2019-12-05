/**
 * @brief Drive brushed DC-motors with TB6612
 * @author botamochi6277
 */

#include "ros/ros.h"
#include <iostream>
// #include <chrono>
// #include <thread>
#include <pigpiod_if2.h>
#include "std_msgs/Empty.h"

class Gun{
  private:
  int pi_;
  int pin_motor_;
  float time_per_shot_;

  Gun(int pi, int pin_motor,float t){
    pi_ = pin;
    pin_motor_ = pin_motor;
    time_per_shot_ = t;
    set_mode(pi_, pin_motor_, PI_OUTPUT);
  }

  void callback(const std_msgs::Empty::ConstPtr& msg){
    gpio_write(pi_, pin_bin1_, 1);
    ros::Duration(time_per_shot_).sleep();
    gpio_write(pi_, pin_bin1_, 0);
  }
}

int main(int argc, char **argv) {

  // establish ros node
  ros::init(argc, argv, "shot");
  ros::NodeHandle nh;

  // connect to pigpio
  int pi;
  pi = pigpio_start(NULL, NULL);

  if (pi < 0) {
    std::printf("cannot connect pigpiod\n");
    return -1;
  }

  // define pin assignment for TB6612
  int pin_motor = 21;

  // Read pin assignment from ROSPARAM
  std::vector<std::string> pinouts;
  if (nh.getParam("pigpio", pinouts)) {
    for (int i = 0; i < pinouts.size(); ++i) {
      if (pinouts.at(i) == "Shoot") {
        pin_ain1 = i;
      }
    }
  } else {
    ROS_ERROR("No pigpio param");
  }

  Gun gun(pi,pin_motor,1.0);
  ros::Subscriber sub = nh.subscribe("tamiya1/shot", 1000, &Gun::callback, &gun);

  ros::spin();

  // disconnect pigpio
  pigpio_stop(pi);
  return 0;
}
// %EndTag(FULLTEXT)%