// Copyright 2020 FTM
#include "ros/ros.h"
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_msgs/SafetyDriverStatus.h"
#include "tod_network/udp_receiver.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <tod_helper/vehicle/Model.h>

struct SafetyDriverStatus {
    bool vehicle_lat_approved;
    bool vehicle_long_approved;
    bool vehicle_emergency_stop_pressed;
};

tod_msgs::SafetyDriverStatus convert_to_ros_msg(const SafetyDriverStatus& safety_driver_status) {
    tod_msgs::SafetyDriverStatus safety_driver_status_msg;
    safety_driver_status_msg.vehicle_emergency_stop_released = !safety_driver_status.vehicle_emergency_stop_pressed;
    safety_driver_status_msg.vehicle_lat_approved = safety_driver_status.vehicle_lat_approved;
    safety_driver_status_msg.vehicle_long_approved = safety_driver_status.vehicle_long_approved;
    return safety_driver_status_msg;
}

void publish_every_tenth_msg(ros::Publisher& publisher, const SafetyDriverStatus& safety_driver_status) {
    static int number_received_safety_driver_status_msgs{ 0 };
    ++number_received_safety_driver_status_msgs;
    if (number_received_safety_driver_status_msgs % 10 == 0) { // autobox runs at 1000 Hz only send data at 100 Hz
        tod_msgs::SafetyDriverStatus safety_driver_status_msg;
        safety_driver_status_msg = convert_to_ros_msg(safety_driver_status);
        publisher.publish(safety_driver_status_msg);
    }
}

void inform_the_user_about_success_once() {
    static bool once{false};
    if (!once) {
        once = true;
        ROS_INFO("%s: Received first autobox status data from autobox!",
            ros::this_node::getName().c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "AutoboxStatusFromAutobox");
    std::string nodeName = ros::this_node::getName();
    ros::NodeHandle n;
    tod_network::UdpReceiver autoboxReceiver(tod_network::VehiclePorts::RX_SAFETY_DRIVER_STATUS_AUTOBOX);
    ros::Publisher pubAutoboxStatus = n.advertise<tod_msgs::SafetyDriverStatus>("safety_driver_status", 1);

    SafetyDriverStatus safety_driver_status;

    while (ros::ok()) {
        // .recvData() is blocking
        int bytesReceived = autoboxReceiver.receive_data((char*) &safety_driver_status);
        publish_every_tenth_msg(pubAutoboxStatus, safety_driver_status);
        inform_the_user_about_success_once();
        ros::spinOnce();
    }
    autoboxReceiver.wait_for_udp_receiver_to_close();
    return 0;
}
