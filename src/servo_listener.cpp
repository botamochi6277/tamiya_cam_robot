#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <TB6612.hpp>
#include <pigpiod_if2.h>


class Servo {
 private:
  int pi_;
  int pin_;

  // Angle Setting
  float min_angle_ = -90.0;
  float max_angle_ = 90.0;

  // Pulse Width Setting
  int min_width_ = 500;
  int max_width_ = 2400;

 public:

  Servo(int pi, int pin) {
    pi_ = pi;
    pin_ = pin;
  }

  /**
   * send reference angle to a servo
   * @param angle reference angle
   */
  void write(float angle) {
    // PWM Frequency : 50Hz(20 ms)
    // pulsewidth 500-2500 [us]
    // SG92R : 500-2400 [us]

    // Limit Angle
    if (angle < min_angle_) {
      angle = min_angle_;
    }
    if (angle > max_angle_) {
      angle = max_angle_;
    }

    int width = (angle - min_angle_) * (max_width_ - min_width_) / (max_angle_ - min_angle_) + min_width_;
    set_servo_pulsewidth(pi_, pin_, width);
  }

  void sleep() {
    set_servo_pulsewidth(pi_, pin_, 0);
  }

  void setAngleRange(int minVal, int maxVal) {
    min_angle_ = minVal;
    max_angle_ = maxVal;
  }

  void setWidthRange(int minVal, int maxVal) {
    min_width_ = minVal;
    max_width_ = maxVal;
  }

  void callback(const std_msgs::Float32::ConstPtr& msg) {
    write(msg->data);
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "servo_listener");
  ros::NodeHandle nh;
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    ROS_ERROR("cannot connect pigpiod\n");
    return -1;
  }

  int pin_servo = 4;
  std::string pin_name = "servo";
  if (node_private.getParam("name", pin_name)) {
    //
  }

  // Read pin assign
  std::vector<std::string> pinouts;
  if (nh.getParam("pigpio", pinouts)) {
    for (int i = 0; i < pinouts.size(); ++i) {
      if (pinouts.at(i) == pin_name) {
        pin_servo = i;
      }
    }
  } else {
    ROS_ERROR("No pigpio param");
  }
  ROS_INFO("Servo info: ");
  ROS_INFO("Servo %s - pin %02d", pin_name.c_str(), pin_servo);

  Servo myservo(pi, pin_servo);

  ros::Subscriber sub = nh.subscribe("tamiya1/" + pin_name, 1000,
                                     &Servo::callback, &myservo);
  ros::spin();
  myservo.sleep();
  pigpio_stop(pi);
  return 0;
}