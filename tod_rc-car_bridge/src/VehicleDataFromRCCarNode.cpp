// Copyright 2020 Florian Sauerbeck

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "tod_msgs/VehicleData.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataFromRCCar");
    std::string nodeName = ros::this_node::getName();
    ros::NodeHandle n;

    tod_msgs::VehicleData msgVehData;
    ros::Subscriber subSteeringWheelAngle = n.subscribe<std_msgs::Float64>(
        "/vesc/commands/servo/position", 1, [&](const std_msgs::Float64ConstPtr &msg) {
            msgVehData.steeringWheelAngle = -((float) msg->data - 0.51);
            ROS_INFO_ONCE("%s: Received first steering wheel angle.", nodeName.c_str());
        });
    ros::Subscriber subLongitudinalSpeed = n.subscribe<nav_msgs::Odometry>(
        "/vesc/odom", 1, [&](const nav_msgs::OdometryConstPtr &msg) {
            msgVehData.longitudinalSpeed = (float) msg->twist.twist.linear.x;
            ROS_INFO_ONCE("%s: Received first longitudinal speed.", nodeName.c_str());
        });
    ros::Subscriber subEngineSpeed = n.subscribe<std_msgs::Float64>(
        "/vesc/commands/motor/speed", 1, [&](const std_msgs::Float64ConstPtr &msg) {
            msgVehData.engineSpeed = (float) msg->data;
            ROS_INFO_ONCE("%s: Received first engine speed.", nodeName.c_str());
        });
    ros::Subscriber subAcceleration = n.subscribe<sensor_msgs::Imu>(
        "/hedge_imu", 1, [&](const sensor_msgs::ImuConstPtr &msg) {
            msgVehData.linearAcceleration = msg->linear_acceleration;
            ROS_INFO_ONCE("%s: Received first linear acceleration.", nodeName.c_str());
        });

    ros::Rate r{100};
    ros::Publisher pubVehicleData = n.advertise<tod_msgs::VehicleData>("vehicle_data", 1);
    while (ros::ok()) {
        ros::spinOnce();
        pubVehicleData.publish(msgVehData);
        r.sleep();
    }

    return 0;
}
