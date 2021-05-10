// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_receiver.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "OdometryReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<nav_msgs::Odometry>
            receiver(n, tod_network::OperatorPorts::RX_VEHICLESTATE_ODOMETRY);
    receiver.run();
    return 0;
}
