// Copyright 2020 Florian Sauerbeck

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "tod_msgs/PrimaryControlCmd.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleCommandToRCCar");
    std::string nodeName = ros::this_node::getName();
    ros::NodeHandle n;
    ros::Publisher pubAckermannDrive = n.advertise<ackermann_msgs::AckermannDriveStamped>(
        "/vesc/low_level/ackermann_cmd_mux/input/navigation", 1);
    ros::Subscriber subPrimControlCommand = n.subscribe<tod_msgs::PrimaryControlCmd>("primary_control_cmd", 1,
        [&](const tod_msgs::PrimaryControlCmdConstPtr &msg) {
            ackermann_msgs::AckermannDriveStamped ackermannMsg;
            ackermannMsg.header.stamp = ros::Time::now();
            ackermannMsg.drive.acceleration = msg->acceleration;
            ackermannMsg.drive.speed = msg->velocity;
            ackermannMsg.drive.steering_angle = msg->steeringWheelAngle;
            pubAckermannDrive.publish(ackermannMsg);
            ROS_INFO_ONCE("%s: Converted first PrimaryControlCmd to AckermannDrive", nodeName.c_str());
        });
    ros::spin();
    return 0;
}
