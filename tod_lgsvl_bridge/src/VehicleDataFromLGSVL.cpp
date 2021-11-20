// Copyright 2021 FTM

#include "ros/ros.h"
#include "tod_msgs/VehicleData.h"
#include "lgsvl_msgs/CanBusData.h"
#include "autoware_msgs/VehicleCmd.h"
#include "sensor_msgs/NavSatFix.h"

// TODO: Refactor this mapping.
enum gearMap {
    NEUTRAL = 0,
    DRIVE = 1,
    REVERSE = 2,
    PARK = 3,
    LOW = 4
};

float _maxSWA{0};

ros::Publisher vehicleDataMsgPub, gpsMsgPub;

void callback_command(const lgsvl_msgs::CanBusDataConstPtr &msg) {

    tod_msgs::VehicleData vehicleData;

    std::vector<int> indicator{static_cast<int>(msg->left_turn_signal_active),
                               static_cast<int>(msg->right_turn_signal_active),
                               static_cast<int>(msg->hazard_lights_active)};

    // Remap of the indicator key-bindings.
    std::vector<int>::iterator it = std::find(indicator.begin(), indicator.end(), 1);
    if (it != indicator.end())
        vehicleData.indicator = std::distance(indicator.begin(), it) + 1;
    else
        vehicleData.indicator = 0;

    // TODO: Refactor.
    switch (msg->selected_gear) {
        case 0:
            vehicleData.gearPosition = 2;
            break;
        case 1:
            vehicleData.gearPosition = 3;
            break;
        case 2:
            vehicleData.gearPosition = 1;
            break;
        case 3:
            vehicleData.gearPosition = 0;
            break;
        case 4:
            vehicleData.gearPosition = 4;
            break;
        default:
            // Default puts the gear to PARK.
            vehicleData.gearPosition = 2;
    }

    vehicleData.engineSpeed = msg->engine_rpm;
    vehicleData.wiper = static_cast<int>(msg->wipers_active);
    vehicleData.headLight = static_cast<int>(msg->high_beams_active);
    vehicleData.flashLight = static_cast<int>(msg->low_beams_active);
    vehicleData.longitudinalSpeed = msg->speed_mps;
    vehicleData.steeringWheelAngle = -_maxSWA * msg->steer_pct;
    vehicleData.header.stamp = ros::Time::now();
    vehicleDataMsgPub.publish(vehicleData);

    sensor_msgs::NavSatFix gpsMsg;
    gpsMsg.header.frame_id = "gps";
    gpsMsg.header.stamp = ros::Time::now();
    gpsMsg.longitude = msg->gps_longitude;
    gpsMsg.latitude = msg->gps_latitude;
    gpsMsg.altitude = msg->gps_altitude;
    gpsMsgPub.publish(gpsMsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataFromLGSVL");
    ros::NodeHandle n;
    ros::Subscriber subControlCommand = n.subscribe("/Simulation/canbus", 1, callback_command);
    vehicleDataMsgPub = n.advertise<tod_msgs::VehicleData>("vehicle_data", 1);
    gpsMsgPub = n.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
    std::string _nodeName = ros::this_node::getName();
    if (!n.getParam(_nodeName + "/maximum_steering_wheel_angle", _maxSWA))
        ROS_ERROR("%s: Missing vehicle parameter!", _nodeName.c_str());


    ros::spin();
    return 0;
}
