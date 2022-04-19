// Copyright 2020 FTM
#include "ros/ros.h"
#include "tod_msgs/PrimaryControlCmd.h"
#include "tod_msgs/SecondaryControlCmd.h"
#include "tod_network/udp_sender.h"

typedef struct tag_tDirectControlOutput {
    bool scheibenwischer;
    bool hupe;
    bool lichthupe;
    float gaspedal;
    float bremspedal;
    float lenkradwinkel;
    float sollgeschwindigkeit;
    float sollbeschleunigung;
    int gangwahl;
    int blinker;
    int64_t stamp;

    tag_tDirectControlOutput() :
        gaspedal(0), bremspedal(0), lenkradwinkel(0), sollgeschwindigkeit(0),
        sollbeschleunigung(0), gangwahl(0), blinker(0), scheibenwischer(false),
        hupe(false), lichthupe(false) {}
} tDirectControlOutput;

tod_network::UdpSender autoboxSender(tod_network::AutoboxIp,
    tod_network::AutoboxPorts::RX_DIRECTCONTROL_AUTOBOX_COMMAND);
tDirectControlOutput controlCommand;

void callback_primary_command(const tod_msgs::PrimaryControlCmdConstPtr& msg) {
    controlCommand.gaspedal = 0.0;
    controlCommand.bremspedal = 0.0;
    controlCommand.lenkradwinkel = msg->steeringWheelAngle;
    controlCommand.sollgeschwindigkeit = msg->velocity;
    controlCommand.sollbeschleunigung = msg->acceleration;
    autoboxSender.send((char*) &controlCommand, sizeof(controlCommand));
}

void callback_secondary_command(const tod_msgs::SecondaryControlCmdConstPtr& msg) {
    controlCommand.scheibenwischer = msg->wiper;
    controlCommand.hupe = msg->honk;
    controlCommand.lichthupe = msg->flashLight;
    controlCommand.gangwahl = msg->gearPosition;
    controlCommand.blinker = msg->indicator;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "ControlCommandToAutobox");
    ros::NodeHandle n;
    ros::Subscriber subPrimControlCommand = n.subscribe("/Vehicle/VehicleBridge/primary_control_cmd",
            1, callback_primary_command);
    ros::Subscriber subSecControlCommand = n.subscribe("/Vehicle/CommandCreation/secondary_control_cmd",
            1, callback_secondary_command);
    ros::spin();
    return 0;
}
