// Copyright 2021 FTM

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

ros::Publisher odometryPub;

double offsetX = 0;
double offsetY = 0;
double offsetZ = 0;
double userOffsetX = 0;
double userOffsetY = 0;
double userOffsetZ = 0;
bool invertY = false;
bool firstOdometryCallback{true};

void odometry_callback(const nav_msgs::Odometry& in) {
    if(firstOdometryCallback){
        offsetX = in.pose.pose.position.x;
        offsetY = in.pose.pose.position.y;
        offsetZ = in.pose.pose.position.z;
        firstOdometryCallback = false;
    }
    
    nav_msgs::Odometry out;
    out = in;
    out.pose.pose.position.x = in.pose.pose.position.x -offsetX + userOffsetX;
    if (invertY)
        out.pose.pose.position.y = -(in.pose.pose.position.y -offsetY) + userOffsetY;
    else
        out.pose.pose.position.y = in.pose.pose.position.y -offsetY + userOffsetY;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "ftm";
    odometryPub.publish(out); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "OdometryOffset");
    ros::NodeHandle n;
    ros::Subscriber odometrySub = n.subscribe("odometry_in", 1, odometry_callback);
    odometryPub = n.advertise<nav_msgs::Odometry>("odometry_out", 1);
    if (!n.getParam(ros::this_node::getName() + "/userOffsetX", userOffsetX))
        ROS_DEBUG("%s: Could not get parameter - using %f",
                ros::this_node::getName().c_str(), userOffsetX);

    if (!n.getParam(ros::this_node::getName() + "/userOffsetY", userOffsetY))
        ROS_DEBUG("%s: Could not get parameter - using %f",
                ros::this_node::getName().c_str(), userOffsetY);

    if (!n.getParam(ros::this_node::getName() + "/userOffsetZ", userOffsetZ))
        ROS_DEBUG("%s: Could not get parameter - using %f",
                ros::this_node::getName().c_str(), userOffsetZ);
    
    if (!n.getParam(ros::this_node::getName() + "/invertY", invertY))
        ROS_DEBUG("%s: Could not get parameter - using %b",
                ros::this_node::getName().c_str(), invertY);
    ros::spin();
    return 0;
}
