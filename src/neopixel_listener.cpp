#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"
#include <iostream>
#include <vector>

#include <pigpiod_if2.h>

class Neopixels {
 private:
  int pi_;
  int num_ = 0;

  std::vector<char> buffs_;
  // GRB hexcolors
  std::vector<unsigned int> colors_;

  int handle_;

 public:
  /**
   * @brief default constructor
   * @param pi pigpio id
   * @param num the number of neopixels
   */
  Neopixels(int pi, int num, int channel = 0, int aux = 0) {
    pi_ = pi;
    num_ = num;
    // initialize vectors
    buffs_.assign(0x0, num_ * 24);// byte array
    colors_.assign(0x0, num_);// hexcolor array

    int flags = 255 * aux;// [aux=0,flags=0], [aux=1,flags=255]

    handle_ = spi_open(pi_, channel, 6400000, flags);

    if (handle_ < 0) {
      ROS_ERROR("Fail to open SPI-%1", aux);
    }
  }

  /**
   * @brief stop communicating
   */
  void stop() {
    buffs_.assign(0x0, num_ * 24);
    show();
    spi_close(pi_, handle_);
  }

  void setColor(int index, int hexcolor) {
    // RGB -> GRB
    unsigned int grb = ((hexcolor & 0xff0000) >> 8) |
                       ((hexcolor & 0x00ff00) << 8) |
                       ((hexcolor & 0x0000ff));
    if (index < colors_.size()) {
      colors_[index] = grb;
    } else {
      ROS_WARN("%d is over the neopixels size (%d).",
               index, colors_.size());
    }
  }

  void setColor(int index, int red, int blue, int green) {
    if (index < colors_.size()) {
      // GRB
      colors_[index] = ((green << 16) | (red << 8) | blue);
    } else {
      ROS_WARN("%d is over the neopixels size (%d).",
               index, colors_.size());
    }
  }

  void fill(unsigned int hexcolor) {
    colors_.assign(hexcolor, num_);
  }

  void fill(int red, int blue, int green) {
    // GRB
    unsigned int c = ((green << 16) | (red << 8) | blue);
    fill(c);
  }

  /**
   * @brief update neopixels colors
   **/
  void show() {
    for (int i = 0; i < num_; ++i) {
      for (int j = 0; j < 24; ++j) {
        // 2進数の1桁を抽出して比較
        // 0b00: 0xE0, 0b11100000
        // 0b01: 0xFB, 0b11111011
        if (((colors_[i] >> (23 - j)) & 0b1) == 0b00) {
          buffs_[24 * i + j] = 0xE0;
        } else {
          buffs_[24 * i + j] = 0xF8;
        }
      }
    }

    spi_write(pi_, handle_, &buffs_.front(), buffs_.size());
    ros::Duration(1.0e-4).sleep(); // waiting for updating colors
  }

  void callback(const std_msgs::UInt32MultiArray::ConstPtr& msg) {
    int len = sizeof msg->data / sizeof msg->data[0];
    for (int i = 0; i < len; ++i) {
      setColor(i, msg->data[i]);
    }
    show();
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "neopixel_listener");
  ros::NodeHandle nh;
  ros::NodeHandle node_private("~");

  int pi;
  pi = pigpio_start(NULL, NULL); /* Connect to Pi. */

  if (pi < 0) {
    std::printf("cannot connect pigpiod\n");
    return -1;
  }

  int num_pixels = 1;
  if (!node_private.getParam("num", num_pixels)) {
    // you have obtained "num"
  }

  Neopixels mypixels(pi, num_pixels);

  ros::Subscriber sub = nh.subscribe("/neopixels", 1000,
                                     &Neopixels::callback, &mypixels);

  pigpio_stop(pi);
  // ROS_INFO("Finish driving.");
  return 0;
}