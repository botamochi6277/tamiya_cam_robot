/**
 * @file servo_listener.cpp
 *
 * @brief ROS Listener to control a RC servo motor with pulse.
 *
 * @nodename servo_listener
 * @subscribing_topic "tamiya1/servo", std_msgs/Float32.h
 * @rosparam pin[int] gpio pin number
**/

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <TB6612.hpp>
#include <pigpiod_if2.h>

/**
 * @brief Servo Controller
 */
class Servo {
 private:
  int pi_;
  int pin_;

  float time_ = 1.0;

  // Angle Setting
  float min_angle_ = -90.0;
  float max_angle_ = 90.0;

  // Pulse Width Setting
  int min_width_ = 500;
  int max_width_ = 2400;

 public:
  /**
   * @brief default constructor
   * @param pi       pigpio id
   * @param pin pin number connected to the servo signal
   */
  Servo(int pi, int pin) {
    pi_ = pi;
    pin_ = pin;
  }

  /**
   * @brief send reference angle to a servo
   * @param angle reference angle [degree]
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

  /**
   * @brief sleep the servo motor
   */
  void sleep() {
    set_servo_pulsewidth(pi_, pin_, 0);
  }

  /**
   * @brief Set angle range of the servo
   * @param minVal min. angle of the servo [degree]
   * @param maxVal max. angle of the servo [degree]
   */
  void setAngleRange(int minVal, int maxVal) {
    min_angle_ = minVal;
    max_angle_ = maxVal;
  }

  /**
   * @brief Set pulse width range of the servo
   * @param minVal min. pulse width of the servo
   * @param maxVal max. pulse width of the servo
   */
  void setWidthRange(int minVal, int maxVal) {
    min_width_ = minVal;
    max_width_ = maxVal;
  }

  float SinInterpolation(float begin, float end, float t) {
    return 0.5f * (end - begin) * sin(M_PI / time_ * t - 0.5f * M_PI) + 0.5f * (end + begin);
  }

  /**
   * @brief callback for ros subscriber
   * @param msg rosmessage
   */
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

  int pin_servo = -1;
  if (!node_private.getParam("pin", pin_servo)) {
    pin_servo = 18; // Not found, set a default value
  }

  float time = 0.0;
  if (!node_private.getParam("time", time)) {
  }

  // ROS_INFO("Servo info: ");
  ROS_INFO("Servo Ready - pin %02d", pin_servo);

  Servo myservo(pi, pin_servo);

  ros::Subscriber sub = nh.subscribe((boost::format("tamiya1/servo%02d") % pin_servo).str(),
                                     1000,
                                     &Servo::callback,
                                     &myservo);
  ros::spin();
  myservo.sleep();
  pigpio_stop(pi);
  return 0;
}