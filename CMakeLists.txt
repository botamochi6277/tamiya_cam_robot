cmake_minimum_required(VERSION 2.8.3)
project(tamiya_cam_robot)
set(CMAKE_CXX_STANDARD 14) # C++14...
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs genmsg)
find_package(Threads REQUIRED)

# add_subdirectory(src)
include_directories(src)
## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


# web_listener
add_executable(web_listener src/web_listener.cpp)
target_link_libraries(web_listener ${catkin_LIBRARIES})
target_link_libraries(web_listener Threads::Threads)
target_link_libraries(web_listener pigpiod_if2)
add_dependencies(web_listener tamiya_cam_robot_generate_messages_cpp)


## test
add_executable(test_run src/test_run.cpp)
target_link_libraries(test_run ${catkin_LIBRARIES})
target_link_libraries(test_run Threads::Threads)
target_link_libraries(test_run pigpiod_if2)
add_dependencies(test_run tamiya_cam_robot_generate_messages_cpp)