// Copyright 2020 Simon Hoffmann
#include "ros/ros.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include "tod_msgs/VehicleEnums.h"
#include "nav_msgs/Odometry.h"
#include "VehicleModel.h"
#include <tod_msgs/VehicleData.h>
#include <tod_msgs/Status.h>
#include "tod_helper/vehicle/Model.h"

static float _swa = 0;
static float _velocity = 0;
static tod_msgs::VehicleData _vehDataMsg;
static VehicleModel _vehModel;

void handlePrimaryControl(const tod_msgs::PrimaryControlCmd& msg) {
    _swa = msg.steeringWheelAngle;
    _velocity = msg.velocity;
}

void handleSecondaryControl(const tod_msgs::SecondaryControlCmd& msg) {
    _vehDataMsg.honk = msg.honk;
    _vehDataMsg.wiper = msg.wiper;
    _vehDataMsg.headLight = msg.headLight;
    _vehDataMsg.indicator = msg.indicator;
    _vehDataMsg.flashLight = msg.flashLight;
    _vehDataMsg.gearPosition = msg.gearPosition;
}

void handleStatus(const tod_msgs::Status &msg) {
    static uint8_t prevConnStatus{tod_msgs::Status::TOD_STATUS_IDLE};
    uint8_t currConnStatus = msg.tod_status;
    if (currConnStatus == tod_msgs::Status::TOD_STATUS_IDLE
        && prevConnStatus != tod_msgs::Status::TOD_STATUS_IDLE) {
        // on disconnect
        _vehModel.resetInitialPosition(0.0, 0.0, 0.0);
    }
    prevConnStatus = currConnStatus;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleSimNode");
    ros::NodeHandle n;

    ros::Subscriber primaryControlSubs = n.subscribe("primary_control_cmd", 1, handlePrimaryControl);
    ros::Subscriber secondaryControlSubs = n.subscribe("secondary_control_cmd", 1, handleSecondaryControl);
    ros::Subscriber statusSubs = n.subscribe("/Vehicle/Manager/status_msg", 1, handleStatus);
    ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Publisher odomPub2 = n.advertise<nav_msgs::Odometry>("/Vehicle/VehicleBridge/odometry_rear", 1);
    ros::Publisher vehDataPub = n.advertise<tod_msgs::VehicleData>("vehicle_data", 1);
    std::string nodeName = ros::this_node::getName();

    // Load Parameters
    float distance_rear_axle{0.0f};
    float distance_front_axle{0.0f};
    float max_steering_wheel_angle{0.0f};
    float max_road_wheel_angle{0.0f};
    if (!n.getParam(nodeName + "/distance_rear_axle", distance_rear_axle) ||
        !n.getParam(nodeName + "/distance_front_axle", distance_front_axle) ||
        !n.getParam(nodeName + "/maximum_road_wheel_angle", max_road_wheel_angle) ||
        !n.getParam(nodeName + "/maximum_steering_wheel_angle", max_steering_wheel_angle)) {
        ROS_ERROR("%s: Could not get Vehicle Parameters", nodeName.c_str());
    }
    _vehModel.setParams(distance_front_axle, distance_rear_axle, max_road_wheel_angle, max_steering_wheel_angle);

    nav_msgs::Odometry base_odom, rear_odom;
    ros::Time prev = ros::Time::now();
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
        ros::Time curr = ros::Time::now();
        double dt_s = (curr-prev).toSec();

        // Calculate vehicle movement
        _vehModel.updatePosition(_velocity, _swa, _vehDataMsg.gearPosition, dt_s);
        _vehDataMsg.longitudinalSpeed = float(_vehModel.getVx());
        if (_vehDataMsg.gearPosition == eGearPosition::GEARPOSITION_REVERSE)
            _vehDataMsg.longitudinalSpeed *= (-1.0f);
        _vehDataMsg.linearAcceleration.x = _vehModel.getAx();
        _vehDataMsg.linearAcceleration.y = _vehModel.getAy();
        _vehDataMsg.steeringWheelAngle = float(tod_helper::Vehicle::Model::rwa2swa(
            _vehModel.getSteeringAngle(), max_steering_wheel_angle, max_road_wheel_angle));
        _vehDataMsg.header.stamp = ros::Time::now();

        // Publishing odometry
        base_odom.header.stamp = ros::Time::now();
        base_odom.header.frame_id = "ftm";
        base_odom.child_frame_id = "base_footprint";
        base_odom.pose.pose.position.x = _vehModel.getX();
        base_odom.pose.pose.position.y = _vehModel.getY();
        base_odom.pose.pose.position.z = 0.0;
        base_odom.twist.twist.angular.z = _vehModel.getPsi_p();
        base_odom.twist.twist.linear.x = _vehModel.getVx();
        base_odom.twist.twist.linear.y = _vehModel.getVy();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, _vehModel.getPsi());
        tf2::convert(q, base_odom.pose.pose.orientation);

        rear_odom = base_odom;
        rear_odom.pose.pose.position.x = rear_odom.pose.pose.position.x
                                         - distance_rear_axle * std::cos(_vehModel.getPsi());
        rear_odom.pose.pose.position.y = rear_odom.pose.pose.position.y
                                         - distance_rear_axle * std::sin(_vehModel.getPsi());
        rear_odom.child_frame_id = "rear_axle_footprint";
        odomPub2.publish(rear_odom);
        odomPub.publish(base_odom);
        vehDataPub.publish(_vehDataMsg);

        prev = curr;
    }
    return 0;
}
