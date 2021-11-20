// Copyright 2021 FTM

#include "ros/ros.h"
#include "tod_msgs/SafetyDriverStatus.h"


void publish_every_tenth_msg(ros::Publisher& publisher) {

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "SafetyDriverStatus");
    ros::NodeHandle n;
    ros::Publisher pubAutoboxStatus = n.advertise<tod_msgs::SafetyDriverStatus>("safety_driver_status", 1);

    tod_msgs::SafetyDriverStatus safety_driver_status_msg;
    ros::Rate rate(100);

    safety_driver_status_msg.vehicle_emergency_stop_released = true;
    safety_driver_status_msg.vehicle_long_approved = true;
    safety_driver_status_msg.vehicle_lat_approved = true;
    
    while (ros::ok()) {
        pubAutoboxStatus.publish(safety_driver_status_msg);
        rate.sleep();
    }

    return 0;
}
