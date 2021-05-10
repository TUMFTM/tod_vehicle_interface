// Copyright 2020 Simon Hoffmann

#include "tod_network/tod_receiver.h"
#include "sensor_msgs/NavSatFix.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "GPSReceiver");
    ros::NodeHandle n;
    tod_network::Receiver<sensor_msgs::NavSatFix>
            receiver(n, tod_network::OperatorPorts::RX_VEHICLESTATE_GPS);
    receiver.run();
    return 0;
}
