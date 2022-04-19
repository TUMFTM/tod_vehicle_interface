// Copyright 2020 FTM
#include "ros/ros.h"
#include "tod_msgs/VehicleData.h"
#include "tod_msgs/VehicleEnums.h"
#include "tod_network/udp_receiver.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <tod_helper/vehicle/Model.h>

typedef struct tag_tFzgData {
    int  gang;
    int  drehzahl;
    float xp;
    float yp;
    float xpp;
    float ypp;
    float psip;
    float lenkradwinkel;
    float radgeschw_hl;
    float radgeschw_hr;
    float radgeschw_vl;
    float radgeschw_vr;
    float radumfang;
    float ersatzgeschw;
    int blinker;
    bool scheibenwischer;
    bool hupe;
    bool lichthupe;
    int64_t stamp;
    tag_tFzgData() : gang(0), drehzahl(0), xp(0), yp(0), xpp(0), ypp(0),
        psip(0), lenkradwinkel(0), radgeschw_hl(0), radgeschw_hr(0),
        radgeschw_vl(0), radgeschw_vr(0), radumfang(0), ersatzgeschw(0),
        blinker(0), scheibenwischer(false), hupe(false), lichthupe(false), stamp(0) {}
} tFzgData;


void fzgDataStructToWheelOdometryMsg(const tFzgData &vehicleData, geometry_msgs::TwistWithCovarianceStamped &msg) {
    static int ctrSend{0};
    msg.header.seq = ++ctrSend;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "rear_wheel_centered_frame";
    msg.twist.twist.linear.x = 1.03*(vehicleData.radgeschw_hl+vehicleData.radgeschw_hr)/2.0;
    msg.twist.twist.linear.y = 0;
    msg.twist.covariance = {0.00012,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.00000001,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0012};
}

void fzgDataStructToVehicleDataMsg(const tFzgData &vehicleData, tod_msgs::VehicleData &msg) {
    static int ctrSend{0};

    msg.header.seq = ++ctrSend;
    msg.header.stamp = ros::Time::now();
    msg.honk = vehicleData.hupe;
    msg.wiper = vehicleData.scheibenwischer;
    msg.curvature = 0.0;
    msg.headLight = HEADLIGHT_OFF;
    msg.indicator = vehicleData.blinker;
    msg.flashLight = vehicleData.lichthupe;
    msg.engineSpeed = vehicleData.drehzahl;
    msg.longitudinalSpeed = tod_helper::Vehicle::Model::kph2mps(vehicleData.xp);
    msg.linearAcceleration.x = vehicleData.xpp;
    msg.linearAcceleration.y = vehicleData.ypp;
    msg.linearAcceleration.z = 0.0;
    msg.steeringWheelAngle = tod_helper::Vehicle::Model::deg2rad(vehicleData.lenkradwinkel);

    switch (vehicleData.gang) {
    case 1: msg.gearPosition = GEARPOSITION_DRIVE; break;
    case 0: msg.gearPosition = GEARPOSITION_PARK; break;
    case 7: msg.gearPosition = GEARPOSITION_REVERSE; break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "VehicleDataFromAutobox");
    std::string nodeName = ros::this_node::getName();
    ros::NodeHandle n;

    tod_network::UdpReceiver autoboxReceiver(tod_network::VehiclePorts::RX_VEHICLEDATA_AUTOBOX);

    ros::Publisher pubVehicleData = n.advertise<tod_msgs::VehicleData>("vehicle_data", 5);
    ros::Publisher pubTwistData = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("vehicle_twist", 5);

    tod_msgs::VehicleData msgVehData;
    geometry_msgs::TwistWithCovarianceStamped twistMsg;

    while (ros::ok()) {
        tFzgData vehicleData;
        int bytesReceived = autoboxReceiver.receive_data((char*) &vehicleData);
        if (bytesReceived > 0) {
            static int ctrRecv{0};
            ++ctrRecv;
            if (ctrRecv % 10 == 0) { // autobox runs at 1000 Hz only send VehicleData at 100 Hz
                fzgDataStructToVehicleDataMsg(vehicleData, msgVehData);
                fzgDataStructToWheelOdometryMsg(vehicleData, twistMsg);

                pubVehicleData.publish(msgVehData);
                pubTwistData.publish(twistMsg);

                static bool once{false};
                if (!once) {
                    once = true;
                    ROS_INFO("%s: Received first vehicle and twist data from autobox!",
                        ros::this_node::getName().c_str());
                }
            }
        } else {
            ROS_ERROR("%s: Received %d Bytes - not publishing VehicleDataMsg!", nodeName.c_str(), bytesReceived);
        }
        ros::spinOnce();
    }

    autoboxReceiver.wait_for_udp_receiver_to_close();
    return 0;
}
