# TAMIYA-CAM-ROBOT

A ROS package to control [Tamiya's Cam-Program Robot](https://www.tamiya.com/english/products/70227/index.htm).

You can teleoperate the robot with web browsers of your smart-phone or tablet.

[Wiki](https://github.com/botamochi6277/tamiya_cam_robot/wiki) of this project provides information helping you.

[![https://www.youtube.com/watch?v=HUGwly77vZY](https://img.youtube.com/vi/HUGwly77vZY/0.jpg)](https://www.youtube.com/watch?v=HUGwly77vZY)

## System Structure

![structure](https://user-images.githubusercontent.com/14128408/77536859-6be32780-6ee0-11ea-8f5b-8a7dac8cfe95.png)

![tamiya_control](https://user-images.githubusercontent.com/14128408/77536866-70a7db80-6ee0-11ea-9f70-31608a7f76ed.png)

| Name | Role                                                                                   | 
|--------------------------------------|--------------------------------------------------------|
| Raspberry Pi | One board PC to control the cam-robot                                              |    
| USB Camera  |Camera to obtain robot's view                                                     |    
| Motor Driver |IC controlling DC motors. This repository uses L298N or TB6612．                                 |  
| R/L-Motor | DC motors of the cam robot                                                                                             
| Remote Device | Device to access the Pi.|                                                 

You need a power bank and alkaline Batteries as power supplies for the Pi and the motors, respectively.

---

- Web Workframe : [Bootstrap v4.3.1](https://getbootstrap.com/)
- Web Theme: [superhero](https://bootswatch.com/superhero/)

