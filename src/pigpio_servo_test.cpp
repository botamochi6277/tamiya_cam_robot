#include <iostream>
#include <pigpiod_if2.h>
#include <chrono>
#include <thread>

// g++ -std=c++11 -Wall -pthread -o pigpio_servo_test pigpio_servo_test.cpp -lpigpiod_if2

class Servo {
 private:
  int pi_;
  int pin_;

  // Angle Setting
  int min_angle_ = -90;
  int max_angle_ = 90;

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
  void write(int angle) {
    // PWM Frequency : 50Hz(20 ms)
    // pulsewidth 500-2500 [us]
    // SG92R : 500-2400 [us]
    unsigned int width = (angle - min_angle_) * (max_width_ - min_width_) / (max_angle_ - min_angle_) + min_width_;
    set_servo_pulsewidth(pi_, pin_, width);
  }

  void setAngleRange(int minVal, int maxVal) {
    min_angle_ = minVal;
    max_angle_ = maxVal;
  }

  void setWidthRange(int minVal, int maxVal) {
    min_width_ = minVal;
    max_width_ = maxVal;
  }

};

int main(int argc, char const *argv[]) {
  // connect to pigpio
  int pi;
  pi = pigpio_start(NULL, NULL);

  if (pi < 0) {
    std::printf("cannot connect pigpiod\n");
    return -1;
  }

  Servo myservo(pi, 25);

  // Sweeeeeeeeeeeeeeeeeeeeeeeeeeep
  for (int angle = -90; angle < 90; ++angle) {
    myservo.write(angle);
    std::this_thread::sleep_for(std::chrono::microseconds(15));
  }

  for (int angle = 90; angle > -90; --angle) {
    myservo.write(angle);
    std::this_thread::sleep_for(std::chrono::microseconds(15));
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  pigpio_stop(pi);

  return 0;
}