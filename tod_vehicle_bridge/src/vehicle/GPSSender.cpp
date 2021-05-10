// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_sender.h"
#include "sensor_msgs/NavSatFix.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "GPSSender");
    ros::NodeHandle n;
    tod_network::Sender<sensor_msgs::NavSatFix> sender(n, tod_network::OperatorPorts::RX_VEHICLESTATE_GPS, true);
    sender.run();
    return 0;
}
