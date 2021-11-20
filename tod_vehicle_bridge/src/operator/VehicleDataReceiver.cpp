// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_receiver.h"
#include "tod_msgs/VehicleData.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<tod_msgs::VehicleData> receiver(n);
    receiver.add_processer("vehicle_data", tod_network::OperatorPorts::RX_VEHICLESTATE_VEHICLEDATA);
    receiver.run();
    return 0;
}
