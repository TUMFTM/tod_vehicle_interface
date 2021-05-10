// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include <iostream>
#include "nav_msgs/Odometry.h"
#include "VehicleModel.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tod_msgs/VehicleData.h>
#include <tod_msgs/Status.h>
#include "tod_helper_functions/VehicleModelHelpers.h"
#include <tf2_ros/transform_broadcaster.h>

uint8_t mode{tod_msgs::Status::CONTROL_MODE_DIRECT};
uint8_t connStatus{tod_msgs::Status::TOD_STATUS_IDLE};

float steeringWheelAngle = 0;
float velocity = 0;
tod_msgs::VehicleData vehDataMsg;

void handlePrimaryControl(const tod_msgs::PrimaryControlCmd& msg) {
    steeringWheelAngle = msg.steeringWheelAngle;
    velocity = msg.velocity;
}

void handleSecondaryControl(const tod_msgs::SecondaryControlCmd& msg) {
    vehDataMsg.honk = msg.honk;
    vehDataMsg.wiper = msg.wiper;
    vehDataMsg.headLight = msg.headLight;
    vehDataMsg.indicator = msg.indicator;
    vehDataMsg.flashLight = msg.flashLight;
    vehDataMsg.gearPosition = msg.gearPosition;
}

void handleStatus(const tod_msgs::Status &msg) {
    mode = msg.vehicle_control_mode;
    connStatus = msg.tod_status;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleSimNode");
    ros::NodeHandle n;
    int loopRate = 100;
    ros::Rate loop_rate(loopRate);

    ros::Subscriber primaryControlSubs = n.subscribe("primary_control_cmd", 1, handlePrimaryControl);
    ros::Subscriber secondaryControlSubs = n.subscribe("secondary_control_cmd", 1, handleSecondaryControl);
    ros::Subscriber statusSubs = n.subscribe("/Vehicle/Manager/status_msg", 1, handleStatus);
    ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Publisher odomPub2 = n.advertise<nav_msgs::Odometry>("/Vehicle/VehicleBridge/odometry_rear", 1);
    ros::Publisher vehDataPub = n.advertise<tod_msgs::VehicleData>("vehicle_data", 1);
    std::string nodeName = ros::this_node::getName();

    // Load Parameters
    float distance_rear_axle;
    float distance_front_axle;
    float max_steering_wheel_angle;
    float max_road_wheel_angle;
    if (!n.getParam(nodeName + "/distance_rear_axle", distance_rear_axle) ||
        !n.getParam(nodeName + "/distance_front_axle", distance_front_axle) ||
        !n.getParam(nodeName + "/maximum_road_wheel_angle", max_road_wheel_angle) ||
        !n.getParam(nodeName + "/maximum_steering_wheel_angle", max_steering_wheel_angle)) {
        ROS_ERROR("%s: Could not get Vehicle Parameters", nodeName.c_str());
    }

    // Create Model
    VehicleModel vehModel = VehicleModel(
        distance_front_axle, distance_rear_axle, max_road_wheel_angle, max_steering_wheel_angle);

    nav_msgs::Odometry base_odom;
    nav_msgs::Odometry rear_odom;
    while (ros::ok()) {
        ros::spinOnce();
        static ros::Time prev = ros::Time::now();
        ros::Time curr = ros::Time::now();
        auto dt = curr - prev;
        double dt_s = double(dt.toNSec()) / 1000000000.0;

        // Calculate vehicle movement
        vehModel.updatePosition(velocity, steeringWheelAngle, dt_s);
        vehDataMsg.longitudinalSpeed = vehModel.getVx();
        vehDataMsg.steeringWheelAngle = VehicleModelHelpers::rwa2swa(vehModel.getSteeringAngle(),
                                                                     max_steering_wheel_angle, max_road_wheel_angle);
        vehDataMsg.header.stamp = ros::Time::now();

        // Publishing odometry
        base_odom.header.stamp = ros::Time::now();
        base_odom.header.frame_id = "ftm";
        base_odom.child_frame_id = "base_footprint";
        base_odom.pose.pose.position.x = vehModel.getX();
        base_odom.pose.pose.position.y = vehModel.getY();
        base_odom.pose.pose.position.z = 0.0;
        base_odom.twist.twist.angular.z = vehModel.getPsi_p();
        base_odom.twist.twist.linear.x = vehModel.getVx();
        base_odom.twist.twist.linear.y = vehModel.getVy();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, vehModel.getPsi());
        base_odom.pose.pose.orientation.x = q.x();
        base_odom.pose.pose.orientation.y = q.y();
        base_odom.pose.pose.orientation.z = q.z();
        base_odom.pose.pose.orientation.w = q.w();

        rear_odom = base_odom;
        rear_odom.pose.pose.position.x = rear_odom.pose.pose.position.x
                                         - distance_rear_axle *std::cos(vehModel.getPsi());
        rear_odom.pose.pose.position.y = rear_odom.pose.pose.position.y
                                         - distance_rear_axle *std::sin(vehModel.getPsi());
        rear_odom.child_frame_id = "rear_axle_footprint";
        odomPub2.publish(rear_odom);
        odomPub.publish(base_odom);
        vehDataPub.publish(vehDataMsg);

        prev = curr;
        loop_rate.sleep();
    }
    return 0;
}
