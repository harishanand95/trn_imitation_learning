//
// Created by Harish on 3/22/20.
//
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;
int main(int argc, char** argv){
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello world" << ros::this_node::getName());
    return 0;
}
