// Copyright 2021 FTM

#include "ros/ros.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "autoware_msgs/VehicleCmd.h"

ros::Publisher autowareMsgPub;

void callback_command(const tod_msgs::PrimaryControlCmdConstPtr& msg) {
 autoware_msgs::VehicleCmd veh_cmd;
    veh_cmd.twist_cmd.twist.linear.x = msg->velocity;
    veh_cmd.twist_cmd.twist.angular.z = msg->steeringWheelAngle;
    autowareMsgPub.publish(veh_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleCommandToLGSVL");
    ros::NodeHandle n;
    ros::Subscriber subControlCommand = n.subscribe("primary_control_cmd", 1, callback_command);
    autowareMsgPub = n.advertise<autoware_msgs::VehicleCmd>("vehicle_cmd", 1);
    ros::spin();
    return 0;
}
