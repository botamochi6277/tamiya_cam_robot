#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"

unsigned int rgbToHex(unsigned int r, unsigned int g, unsigned int b) {
    return ((r << 16) | (g << 8) | b);
}

/**
* @brief Tune a rgb led based on HSV model
* HSV色空間に基づいて,RGBLEDを光らせる
* @param H Hue(色相), 0.0--360.0
* @param S Saturation(彩度), 0.0--1.0
* @param V Value(Brightness, 明度), 0.0--1.0
*/
unsigned int hsvToHex(float h, float s, float v) {
    unsigned int r, g, b;
    int hi;
    float f, p, q, t;
    if (h > 360.0){
        h = (int)h%360;
    }
    hi = ((int)(h / 60)) % 6;
    f = h / 60 - hi;
    p = v * (1 - s);
    q = v * (1 - f * s);
    t = v * (1 - (1 - f) * s);

    // convert to 256 levels
    v *= 255;
    p *= 255;
    q *= 255;
    t *= 255;
    switch (hi) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    case 5: r = v; g = p; b = q; break;
    }

    return rgbToHex(r, g, b);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rainbow_talker");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::UInt32MultiArray>("neopixels", 100);
    ros::Rate loop_rate(1);
    float hue = 0.0;
    while (ros::ok()) {
        hue += 30.0;
        std_msgs::UInt32MultiArray array;
        array.data.resize(2);
        array.data[0] = hsvToHex(hue,1.0,0.8);
        array.data[1] = hsvToHex(hue + 180.0,1.0,0.8);

        pub.publish(array);
        // ROS_INFO("I published array!");
        ros::spinOnce();
        loop_rate.sleep();
    }
}