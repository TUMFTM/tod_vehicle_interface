// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_sender.h"
#include "tod_msgs/VehicleData.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataSender");
    ros::NodeHandle n;
    tod_network::Sender<tod_msgs::VehicleData> sender(n, tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA, true);
    sender.run();
    return 0;
}
