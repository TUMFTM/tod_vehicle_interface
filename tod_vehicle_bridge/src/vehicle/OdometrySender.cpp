// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_sender.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OdometrySender");
    ros::NodeHandle n;
    tod_network::Sender<nav_msgs::Odometry> sender(n, tod_network::OperatorPorts::RX_VEHICLESTATE_ODOMETRY, true);
    sender.run();
    return 0;
}
