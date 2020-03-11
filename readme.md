# TAMIYA-CAM-ROBOT

A ROS package to control [Tamiya's Cam-Program Robot](https://www.tamiya.com/english/products/70227/index.htm).

You can teleoperate the robot with web browsers of your smart-phone or tablet.

[Wiki](https://github.com/botamochi6277/tamiya_cam_robot/wiki) of this project provides information helping you.

![](https://img.youtube.com/vi/QDSQZUwj9yU/0.jpg)](https://www.youtube.com/watch?v=QDSQZUwj9yU)

## System Structure

![](img/structure.png)

![Electric Structure](img/tamiya_control.png)

| Name | Role                                                                                   | 
|--------------------------------------|--------------------------------------------------------|
| Raspberry Pi | One board PC to control the cam-robot                                              |    
| USB Camera  |Camera to obtain robot's view                                                     |    
| Motor Driver |IC controlling DC motors. This repository uses [TB6612](https//www.switch-science.com/catalog/3586/)．                                 |  
| R/L-Motor | DC motors of the cam robot                                                                                             
| Remote Device | Device to access the Pi.|                                                 

You need a power bank and alkaline Batteries as power supplies for the Pi and the motors, respectively.

---

- Web Workframe : [Bootstrap v4.3.1](https://getbootstrap.com/)
- Web Theme: [superhero](https://bootswatch.com/superhero/)

